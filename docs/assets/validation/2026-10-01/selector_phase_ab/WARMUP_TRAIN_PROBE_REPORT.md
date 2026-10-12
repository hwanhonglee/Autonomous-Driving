# HH_261001 — Offline TRAIN report command

The offline fixed-279-probe reporter is separate from the owned live PNG/GIF reporter. It reads only completed result JSON, has no model forwards or optimizer steps, and requires both endpoints to exist.

```bash
python3 tools/e2e/report_warmup_supervision_train_probes_20261001.py \
  --training storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1 \
  --output docs/assets/validation/2026-10-01/selector_phase_ab/warmup_pair_probe_report.json
```

Its tests are `learning/tests/test_report_warmup_supervision_train_probes_20261001.py`. The file `tools/e2e/report_warmup_supervision_pair_20261001.py` belongs to the separate live evidence/visualization workflow; do not use it for this offline command.

The frozen baseline has 279 probes: 18 original immediate starts, 200 protected STOP/full-hold examples, and 16 anchors in the 1.5–2.5 m/s band (groups can overlap with other fixed boundary probes). Original AH agrees with the unchanged full-cost oracle on all 16 moving-band observations. Report this band separately from the starts; neither a TRAIN probe gain nor an arbitrary positive-speed mode is proof of a closed-loop cure.
