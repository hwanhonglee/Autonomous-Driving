#!/usr/bin/env python3
"""HH_261001 - Explicit 150-second W1 study; retain the original shorter owners and evidence."""

from scripts.e2e.run_owned_ah_town07_research_20261001 import *
from scripts.e2e import run_owned_ah_town07_research_20261001 as original
from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcomes
from scripts.e2e import collect_w1_town07_long_20261001 as worker
from scripts.e2e import collect_ah_simulation_research as short
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity

OUTPUT = worker.OUTPUT
WEIGHTS = ROOT / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports/W1_warmup_anchor_off.npz'
METADATA = WEIGHTS.with_suffix('.json')
PLAN = ROOT / 'docs/assets/validation/2026-10-01/warmup_supervision_ab/LONG_RUN_PLAN.md'
PINS = {path: sha for path, sha in original.PINS.items() if path not in (original.WEIGHTS, original.METADATA)}
PINS.update({WEIGHTS: worker.WEIGHTS_SHA, METADATA: worker.METADATA_SHA})


def collector_command():
    argv = original.collector_command()
    argv[argv.index('scripts.e2e.collect_ah_simulation_research')] = 'scripts.e2e.collect_w1_town07_long_20261001'
    argv[argv.index('--max-duration-sec') + 1] = '150'
    return argv


def prepare():
    # HH_261001 - Resolve bindings in this process only; no historical source is edited.
    original.OUTPUT, original.WEIGHTS, original.METADATA, original.PINS = OUTPUT, WEIGHTS, METADATA, PINS
    argv = collector_command()
    PINS[PLAN] = digest(PLAN)
    plan = original.preflight()
    plan.update(schema='portable_e2e.owned_w1_town07_long.v1',
        collector_command=argv, capture_budget_seconds=150, wall_timeout_seconds=900,
        source_checkpoint_sha256=worker.CHECKPOINT, preregistered_plan_sha256=PINS[PLAN],
        exact_inputs=dict(inference_indices='0,10,20,...', all_inferences_archived=False),
        interpretation='One longer known TRAIN-route diagnostic; unchanged W1 policy, no learned goal-stop claim.')
    extra_start = argv.index(str(OUTPUT / 'episode'))
    args = short.collector.parse_args(argv[extra_start:])
    args.research_weights_sha256, args.research_metadata_sha256 = worker.WEIGHTS_SHA, worker.METADATA_SHA
    args.research_speed_cap_mps, args.research_max_cte_m = 2.5, 1.
    worker.validate(args, json.loads(ROUTE.read_text()))
    return plan


def execute(plan):
    with lifecycle._workspace_lock():
        lifecycle._assert_no_other_carla()
        ports_free(PORT)
        if OUTPUT.exists() or OUTPUT.is_symlink():
            raise RuntimeError("fresh diagnostic output required")
        if shutil.disk_usage(ROOT).free < 5 * 1024**3:
            raise RuntimeError("less than 5 GiB available")
        OUTPUT.mkdir(parents=True)
        imported = {Path(module.__file__).resolve() for module in list(sys.modules.values())
                    if getattr(module, "__file__", None) and Path(module.__file__).is_file()
                    and Path(module.__file__).resolve().is_relative_to(ROOT)}
        sources = sorted(imported | set(ROOT.glob("learning/portable_e2e/*.py"))
            | set(ROOT.glob("learning/portable_e2e/config/*.json"))
            | {ROOT / "tools/e2e/run_carla_map.sh", ROOT / "tools/e2e/env.sh", MAPPING, CALIBRATION, ROUTE, PLAN})
        hashes = {}
        for path in sources:
            relative = path.relative_to(ROOT)
            target = OUTPUT / "source" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with path.open("rb") as src, target.open("xb") as dest:
                shutil.copyfileobj(src, dest)
            hashes[str(relative)] = digest(path)
        plan["source_sha256"] = hashes
        write_json(OUTPUT / "owner_plan.json", plan)
        server = worker = None
        server_identity = None
        error = None
        cleanup_errors = []
        worker_code = None
        prior = {}
        def interrupted(signum, frame):
            raise RuntimeError("owner interrupted by signal " + str(signum))
        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                prior[signum] = signal.signal(signum, interrupted)
            with (OUTPUT / "server.log").open("xb") as log:
                server = subprocess.Popen(plan["server_command"], cwd=ROOT, env=environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            server_identity = process_record(server.pid)
            deadline = time.monotonic() + 130
            socket_owner = None
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError("owned packaged server exited before readiness")
                if b"CARLA_READY " in (OUTPUT / "server.log").read_bytes():
                    children = descendants(server.pid, all_processes())
                    owners = [r for r in children if r["pgid"] == server.pid
                        and any("CarlaUE4-Linux-Shipping" in a for a in r["argv"])
                        and listening_socket_owned(r["pid"], PORT)]
                    if len(owners) != 1:
                        raise RuntimeError("RPC socket is not uniquely owned by the started shipping server")
                    socket_owner = owners[0]
                    break
                time.sleep(1.)
            if socket_owner is None:
                raise RuntimeError("owned packaged server readiness timeout")
            write_json(OUTPUT / "owner_started.json", dict(server=server_identity, socket_owner=socket_owner))
            with (OUTPUT / "collector.log").open("xb") as log:
                worker = subprocess.Popen(plan["collector_command"], cwd=ROOT, env=environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                worker_code = worker.wait(timeout=900)
            if worker_code:
                error = "collector returned " + str(worker_code) + "; bounded route noncompletion or other failure retained"
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            for signum, handler in prior.items():
                signal.signal(signum, handler)
            for process in (worker, server):
                try:
                    if process is server and server_identity:
                        now = process_record(server.pid)
                        if now and now["start_ticks"] != server_identity["start_ticks"]:
                            raise RuntimeError("server PID generation changed; refusing signal")
                    if not lifecycle._stop_owned_group(process):
                        raise RuntimeError("owned process group did not stop")
                except Exception as exc:
                    cleanup_errors.append(str(exc))
            try:
                ports_free(PORT)
                for relative, sha in hashes.items():
                    if digest(ROOT / relative) != sha or digest(OUTPUT / "source" / relative) != sha:
                        raise RuntimeError("source changed during diagnostic: " + relative)
                for path, expected in PINS.items():
                    if digest(path) != expected:
                        raise RuntimeError("pinned asset changed during diagnostic")
            except Exception as exc:
                cleanup_errors.append(str(exc))
            result = dict(status=("OWNER_FAILURE" if cleanup_errors else
                                  "EXECUTION_FAILED_RETAINED" if error else "FINISHED_RESEARCH_ONLY"),
                collector_exit_code=worker_code, error=error, cleanup_errors=cleanup_errors,
                ports_released=not cleanup_errors, training_data_approved=False, vehicle_control_approved=False)
            write_json(OUTPUT / "owner_result.json", result)
        print(json.dumps(result, indent=2))
        return 0 if not cleanup_errors and worker_code == 0 else 1


def final_report():
    result = outcomes.final_report(OUTPUT)
    episode = OUTPUT / 'episode.partial'
    if not episode.exists():
        episode = OUTPUT / 'episode'
    manifest = json.loads((episode / 'manifest.json').read_text()) if episode.exists() else {}
    budget = (manifest.get('status') == 'failed' and manifest.get('error') == outcomes.BUDGET_ERROR
        and result['state_count'] == 3000 and result['camera_count'] == 1500
        and manifest.get('capture_contract', {}).get('maximum_total_duration_sec') == 150.)
    if result['status'] != 'OWNER_FAILURE' and budget:
        result['status'] = 'DIAGNOSTIC_BUDGET_REACHED'
    result.update(schema='portable_e2e.owned_w1_town07_long_outcome.v1',
        source_checkpoint_sha256=worker.CHECKPOINT, learned_goal_stop_claimed=False,
        exact_inputs_all_inferences_archived=False, nominal_total_simulation_budget_seconds=150)
    write_json(OUTPUT / 'owner_diagnostic_result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    with identity(worker.CHECKPOINT, short):
        plan = prepare()
        if not args.execute:
            print(json.dumps(plan, indent=2))
            return 0
        execute(plan)
        result = final_report()
        print(json.dumps(result, indent=2))
        return 0 if result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
