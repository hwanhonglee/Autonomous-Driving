"""HH_260906 - Train one fixed 7:1 Town01/Town03 rehearsal arm; never approve deployment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


PARENT_COUNT = 497
TRAIN_COUNT = 913
DEV_COUNT = 118
PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
PACKAGE_SHA256 = "e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d"
TRAIN_SHA256 = "1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440"
DEV_SHA256 = "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"
B_SOURCE_SHA256 = "86e583aacf654d78ce913a95ba537e0025317aa5a737aefacc2162aa7e2adceb"
B_CHECKPOINT_SHA256 = "12c63095e1a6c401e7772ddf9441c727e3e96ee523623995f1368bf818826fff"
B_SUPPORT_SHA256 = {
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "map_expansion_contract.py": "6dd294564231ceb6fdee8351be5ad27efdf954f06da5b64a8754b9d8dd2fa190",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}
B_TRAIN_PIXELS = [78027023, 99206666, 26647065, 2834271, 178736, 67934, 3393505]
B_WEIGHTS = [0.15812161246466364, 0.14027103630152804, 0.26987722238555106,
             0.8015514213025802, 2.2421913583106625, 2.651247075868451,
             0.736740273366564]


class TownRehearsalSampler:
    """HH_260906 - Draw 7 old and 1 added TRAIN indices with replacement, then shuffle."""

    def __init__(self, parent_count: int = PARENT_COUNT, total_count: int = TRAIN_COUNT):
        if parent_count != PARENT_COUNT or total_count != TRAIN_COUNT:
            raise ValueError("frozen 497/913 TRAIN partition required")
        self.parent_count = parent_count
        self.total_count = total_count

    def sample_batch(self, rng):
        import numpy as np
        old = rng.integers(0, self.parent_count, size=7)
        added = rng.integers(self.parent_count, self.total_count, size=1)
        return rng.permutation(np.concatenate((old, added))).astype(np.int64)

    def contract(self):
        return dict(schema="town01_town03_rehearsal_7_to_1.v1", source="TRAIN indices only",
                    parent_range=[0, self.parent_count], added_range=[self.parent_count, self.total_count],
                    parent_draws_per_batch=7, added_draws_per_batch=1, batch_size=8,
                    replacement=True, shuffled_within_batch=True, seed=20260922,
                    dev_or_test_used=False)


def validate_b_reference(reference, package, old_manifest, digest, require):
    """HH_260906 - Bind C to B's real 913-frame protocol, not B's learned weights."""
    started = json.loads((reference / "started.json").read_text())
    loss = json.loads((reference / "loss_contract.json").read_text())
    summary = json.loads((reference / "summary.json").read_text())
    require(digest(reference / "source/train_map_native_ah.py") == B_SOURCE_SHA256,
            "B training source changed")
    for name, expected in B_SUPPORT_SHA256.items():
        require(digest(reference / "source" / name) == expected,
                "B support source changed")
        require(digest(Path(__file__).with_name(name)) == expected,
                "C support source differs from B")
    require(digest(reference / "checkpoint.pt") == B_CHECKPOINT_SHA256
            and summary.get("checkpoint_sha256") == B_CHECKPOINT_SHA256, "B checkpoint changed")
    require(digest(reference / "data_manifest.json") == PACKAGE_SHA256
            and started.get("data_manifest_sha256") == PACKAGE_SHA256
            and package == json.loads((reference / "data_manifest.json").read_text()),
            "B package changed")
    require(started.get("source_sha256") == B_SOURCE_SHA256
            and started.get("parent_checkpoint_sha256") == PARENT_CHECKPOINT_SHA256
            and started.get("steps") == 2000 and started.get("batch_size") == 8
            and started.get("seed") == 20260922 and started.get("learning_rate") == 3e-4
            and started.get("initialization") == "scratch"
            and started.get("augmentation") == "TRAIN only: probability 0.5 aligned crop, scale 0.65..1.0, resize to native; no flip"
            and started.get("train_count") == TRAIN_COUNT and started.get("dev_count") == DEV_COUNT,
            "B optimization contract changed")
    require(loss.get("dev_used") is False and loss.get("train_pixels") == B_TRAIN_PIXELS
            and loss.get("weights") == B_WEIGHTS, "B loss contract changed")
    require(old_manifest["splits"]["train"]["count"] == PARENT_COUNT,
            "original Town01 TRAIN count changed")
    return started, loss


def main():
    p = argparse.ArgumentParser()
    for name in ("frozen-repo", "data", "parent", "reference-b", "output"):
        p.add_argument("--" + name, type=Path, required=True)
    p.add_argument("--validate-only", action="store_true")
    a = p.parse_args()
    sys.path.insert(0, str(a.frozen_repo))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import numpy as np
    import torch
    from PIL import Image
    from map_perception import MapPerception, LABELS, semantic_loss
    from map_learning_balance import class_weights
    from map_expansion_contract import validate_expansion
    from dual_recovery_objectives import paired_crop
    from scripts.e2e import run_moving_multiroute_recovery_k as r

    workspace = r.check_run_environment(dict(repo_root=str(a.frozen_repo), output_root=str(a.output)))
    r.require(not a.output.exists(), "fresh output required")
    for name in ("data", "parent", "reference_b"):
        r.require(getattr(a, name).resolve().is_relative_to(workspace.parent), "personal input only")
    r.require(r.digest(a.data / "manifest.json") == PACKAGE_SHA256, "913 manifest changed")
    manifest = r.read_json(a.data / "manifest.json")
    parent_summary = r.read_json(a.parent / "summary.json")
    parent_checkpoint = a.parent / "checkpoint.pt"
    r.require(r.digest(a.parent / "data_manifest.json") == PARENT_MANIFEST_SHA256
              and r.digest(parent_checkpoint) == PARENT_CHECKPOINT_SHA256
              and parent_summary["checkpoint_sha256"] == PARENT_CHECKPOINT_SHA256,
              "AI v2 parent changed")
    old_manifest = r.read_json(a.parent / "data_manifest.json")
    validate_expansion(manifest, old_manifest, PARENT_MANIFEST_SHA256)
    r.require(tuple(manifest["labels"]) == LABELS and manifest["image_size_wh"] == [640, 360],
              "native seven-class RGB schema required")
    r.require(manifest["splits"]["train"]["count"] == TRAIN_COUNT
              and manifest["splits"]["dev"]["count"] == DEV_COUNT,
              "frozen TRAIN/DEV counts changed")
    b_started, b_loss = validate_b_reference(a.reference_b, manifest, old_manifest, r.digest, r.require)
    loaded = {}
    for split, expected in (("train", TRAIN_SHA256), ("dev", DEV_SHA256)):
        ref = manifest["splits"][split]
        path = a.data / ref["file"]
        r.require(path.resolve().is_relative_to(a.data.resolve())
                  and ref["sha256"] == expected and r.digest(path) == expected,
                  "TRAIN/DEV byte hash changed")
        with np.load(path, allow_pickle=False) as data:
            x, y = data["rgb"].copy(), data["target"].copy()
        r.require(x.shape == (ref["count"], 360, 640, 3)
                  and y.shape == (ref["count"], 360, 640)
                  and x.dtype == np.uint8 and y.dtype == np.uint8
                  and y.max() < len(LABELS), "array schema changed")
        loaded[split] = x, y
    pixels = np.bincount(loaded["train"][1].ravel(), minlength=len(LABELS))
    weights = class_weights(pixels, "sqrt_frequency")
    r.require(pixels.tolist() == b_loss["train_pixels"]
              and weights.tolist() == b_loss["weights"], "B and C loss weights differ")
    sampler = TownRehearsalSampler(len(old_manifest["splits"]["train"]["source_records"]),
                                   len(loaded["train"][0]))
    if a.validate_only:
        print("TOWN_REHEARSAL_CPU_PREFLIGHT_PASS", flush=True)
        return

    with r.gpu0_lease(workspace):
        # HH_260906 - A busy physical GPU0 is a stop condition; GPU1 is never a fallback.
        gpu = r.preflight_gpu()
        device = torch.device("cuda:0")
        r._seed_everything(20260922, device)
        a.output.mkdir(parents=True)
        for name in ("train_map_town_rehearsal_c.py", "map_perception.py",
                     "map_learning_balance.py", "map_expansion_contract.py",
                     "dual_recovery_objectives.py"):
            r.copy_new(Path(__file__).with_name(name), a.output / "source" / name)
        r.copy_new(a.data / "manifest.json", a.output / "data_manifest.json")
        declaration = dict(gpu=gpu, started_at_utc=r.utc_now(), steps=2000, batch_size=8,
                           seed=20260922, learning_rate=3e-4, input_shape=[3, 360, 640],
                           initialization="scratch", source_sha256=r.digest(Path(__file__)),
                           data_manifest_sha256=PACKAGE_SHA256,
                           parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA256,
                           b_reference_checkpoint_sha256=B_CHECKPOINT_SHA256,
                           b_reference_source_sha256=B_SOURCE_SHA256, loss_mode="sqrt_frequency",
                           augmentation=b_started["augmentation"],
                           changes="TRAIN-only 7:1 Town01/Town03 rehearsal sampler; all B data/loss/model/optimizer/seed/steps fixed",
                           sampling_contract=sampler.contract(), optimizer="AdamW(lr=3e-4,weight_decay=1e-4)",
                           dev_used_for_sampler=False, automatic_promotion=False)
        r.write_new(a.output / "started.json", declaration)
        r.write_new(a.output / "loss_contract.json", dict(train_pixels=pixels.tolist(),
                                                           weights=weights.tolist(), dev_used=False))
        model = MapPerception().to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=3e-4, weight_decay=1e-4)
        weights = torch.tensor(weights, dtype=torch.float32, device=device)
        x, y = loaded["train"]
        sample_rng = np.random.default_rng(20260922)
        augmentation = np.random.default_rng(20260926)
        start = time.monotonic()
        model.train()
        with (a.output / "history.jsonl").open("x") as log:
            for step in range(1, 2001):
                ids = sampler.sample_batch(sample_rng)
                pairs = [paired_crop(x[i], y[i], augmentation) for i in ids]
                images = torch.tensor(np.stack([pair[0] for pair in pairs]).transpose(0, 3, 1, 2).copy(),
                                      device=device, dtype=torch.float32) / 255
                target = torch.tensor(np.stack([pair[1] for pair in pairs]).astype(np.int64), device=device)
                optimizer.zero_grad(set_to_none=True)
                loss = semantic_loss(model(images), target, weights)
                r.require(bool(torch.isfinite(loss)), "nonfinite loss")
                loss.backward()
                norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                r.require(bool(torch.isfinite(norm)), "nonfinite gradient")
                optimizer.step()
                if step % 100 == 0:
                    row = dict(step=step, loss=float(loss.detach()), wall_seconds=time.monotonic() - start)
                    log.write(json.dumps(row) + "\n")
                    log.flush()
                    print(json.dumps(row), flush=True)
        train_seconds = time.monotonic() - start
        r.save_new(a.output / "checkpoint.pt", dict(model_state_dict=model.state_dict(), labels=LABELS,
            model_id="portable_e2e.map_semantics.native_town_rehearsal_c.v1",
            optimizer_state_dict=optimizer.state_dict(), steps=2000, input_shape=[3, 360, 640],
            deployable=False, research_contract=declaration))

        def evaluate(network, examples, labels):
            """HH_260906 - Score the unchanged unaugmented Town01 DEV; this is not TEST."""
            network.eval()
            confusion = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
            predictions = []
            with torch.no_grad():
                for begin in range(0, len(examples), 8):
                    batch = torch.tensor(examples[begin:begin + 8].transpose(0, 3, 1, 2).copy(),
                                         device=device, dtype=torch.float32) / 255
                    pred = network(batch).argmax(1).cpu().numpy().astype(np.uint8)
                    predictions.append(pred)
                    confusion += np.bincount((labels[begin:begin + 8].astype(np.int64) * len(LABELS) + pred).ravel(),
                                             minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS))
            union = confusion.sum(0) + confusion.sum(1) - np.diag(confusion)
            iou = {name: float(confusion[i, i] / union[i]) if union[i] else None
                   for i, name in enumerate(LABELS)}
            return dict(iou=iou, mean_iou_present=float(np.mean([v for v in iou.values() if v is not None])),
                        pixel_accuracy=float(np.trace(confusion) / confusion.sum()),
                        confusion=confusion.tolist()), np.concatenate(predictions)

        model.eval()
        x, y = loaded["dev"]
        native, prediction = evaluate(model, x, y)
        np.savez_compressed(a.output / "dev_predictions.npz", prediction=prediction)
        resample = getattr(Image, "Resampling", Image)
        small_x = np.stack([np.asarray(Image.fromarray(v).resize((320, 180), resample.BILINEAR)) for v in x])
        small_y = np.stack([np.asarray(Image.fromarray(v).resize((320, 180), resample.NEAREST)) for v in y])
        common, _ = evaluate(model, small_x, small_y)
        parent = MapPerception().to(device)
        parent.load_state_dict(torch.load(parent_checkpoint, map_location="cpu", weights_only=False)["model_state_dict"],
                               strict=True)
        baseline_native, _ = evaluate(parent, x, y)
        baseline_common, _ = evaluate(parent, small_x, small_y)
        r.write_new(a.output / "summary.json", dict(status="PILOT_COMPLETE_NOT_PROMOTED", **native,
            steps=2000, training_wall_seconds=train_seconds, wall_seconds=time.monotonic() - start,
            checkpoint_sha256=r.digest(a.output / "checkpoint.pt"), parent_native=baseline_native,
            candidate_common_320=common, parent_common_320=baseline_common,
            model_parameters=sum(parameter.numel() for parameter in model.parameters()),
            sampling_contract=sampler.contract(), b_reference_checkpoint_sha256=B_CHECKPOINT_SHA256,
            limitations="Repeated Town01 DEV; sealed Town07 TEST not inspected; RGB segmentation only, no complete HD map",
            deployment_approved=False, completed_at_utc=r.utc_now()))
        print("TOWN_REHEARSAL_COMPLETE_NOT_PROMOTED", flush=True)


if __name__ == "__main__":
    main()
