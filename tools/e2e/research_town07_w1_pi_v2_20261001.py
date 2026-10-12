"""HH_261001 - Retain v1 shutdown failure; require natural port quiescence in fresh PI attempts."""

from contextlib import contextmanager
import errno
import json
import time

from scripts.e2e import research_town07_w1_pi_20261001 as prior
from scripts.e2e import run_owned_ah_town07_research_20261001 as owner


RUN_ROOT = prior.ROOT / "storage/experiments/validation/2026-10-01/town07_w1_pi_controller_v2"
PLAN = RUN_ROOT.with_name("town07_w1_pi_controller_plan_v2.json")
SCHEMA = "portable_e2e.town07_w1_pi_controller.20261001.v2"
MODULE = "scripts.e2e.research_town07_w1_pi_v2_20261001"
EXTRA = ("tools/e2e/research_town07_w1_pi_v2_20261001.py",
         "learning/tests/test_town07_w1_pi_owner_v2_20261001.py")


def wait_for_original_bind_checks(port, probe, *, timeout_s=10., pause=None, clock=None):
    """HH_261001 - Wait for actual original TCP/UDP binds; never reuse, ignore, or steal a socket."""
    pause, clock = pause or time.sleep, clock or time.monotonic
    started = clock()
    attempts = []
    while True:
        try:
            probe(port)
        except OSError as error:
            attempts.append(dict(elapsed_s=clock()-started, passed=False,
                                 errno=error.errno, error=str(error)))
            if error.errno != errno.EADDRINUSE or clock()-started >= timeout_s:
                raise
            pause(.2)
        else:
            attempts.append(dict(elapsed_s=clock()-started, passed=True))
            return dict(port_triplet=[port, port+1, port+2], maximum_wait_s=timeout_s,
                        original_all_tcp_udp_bind_check=True, sockets_forced_or_reused=False,
                        attempts=attempts)


@contextmanager
def activated():
    changes = dict(RUN_ROOT=RUN_ROOT, PLAN=PLAN, SCHEMA=SCHEMA, MODULE=MODULE,
                   EXTRA_SOURCES=tuple(dict.fromkeys((*prior.EXTRA_SOURCES, *EXTRA))))
    saved = {name: getattr(prior, name) for name in changes}
    old_probe = owner.ports_free
    checks = []

    def probe(port):
        checks.append(wait_for_original_bind_checks(port, old_probe))

    try:
        for name, value in changes.items():
            setattr(prior, name, value)
        owner.ports_free = probe
        with prior.activated():
            # HH_261001 - Freeze the lifecycle wait change before either fresh v2 physical attempt.
            fixed = prior.base.fixed_contract

            def contract():
                return dict(fixed(), original_port_bind_check_natural_release_wait_s=10.,
                    previous_owner_failure_retained=str(prior.ROOT / "storage/experiments/validation/2026-10-01/town07_w1_pi_controller_v1/A1_original"))

            prior.base.fixed_contract = contract
            yield checks
    finally:
        owner.ports_free = old_probe
        for name, value in saved.items():
            setattr(prior, name, value)


def main(argv=None):
    with activated() as checks:
        result = prior.runner.main(argv)
        # HH_261001 - Additional lifecycle receipt never overwrites the authoritative owner failure/result.
        values = list(argv or __import__("sys").argv[1:])
        if "execute" in values and "--trial" in values:
            index = int(values[values.index("--trial")+1])
            path = prior.base.output(index) / "natural_port_release_receipt.json"
            with path.open("x") as stream:
                json.dump(dict(schema=SCHEMA+".natural_release", checks=checks,
                    ports_forced_or_reused=False, other_jobs_terminated=False), stream, indent=2)
        return result


if __name__ == "__main__":
    raise SystemExit(main())
