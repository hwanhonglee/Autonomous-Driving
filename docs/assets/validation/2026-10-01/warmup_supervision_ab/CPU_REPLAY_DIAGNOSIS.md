# W0/W1 exact-input CPU replay diagnosis

<!-- HH_261001 - Preserve the original failures and distinguish input identity from backend-equivalent replay. -->

Resolved without changing the model, input values, validator, or exact-equality rule:
the unchanged validator reproduces **183/183 W0 and 183/183 W1** canonical output
hashes under the live collector's process-start environment. Every maximum
absolute error is zero. The original two `exact_input_replay.json` files remain
`REPLAY_MISMATCH`, with their hashes checked before and after the diagnosis.

The local PyTorch 1.8.0a0 build uses generic BLAS backed by pthread OpenBLAS,
not MKL. `torch.set_num_threads(2)` changes Torch/OpenMP threads but leaves the
separate OpenBLAS pools at 24 threads when startup environment variables are
unset. The live owner explicitly supplies `OMP_NUM_THREADS=2` and
`OPENBLAS_NUM_THREADS=2`. Both loaded BLAS pools then report two threads.

| Fresh process startup | Torch threads during forward | BLAS threads | Exact first-three inputs, both arms, three ownership modes |
| --- | --- | --- | --- |
| OMP/OpenBLAS unset | 2 | 24 | 0/18 |
| OMP=2 only | 2 | 2 | 18/18 |
| OpenBLAS=2 only | 2 | 2 | 18/18 |
| Both=2, live settings | 2 | 2 | 18/18; full unchanged validator 366/366 |
| Both=2, MKLDNN disabled | 2 | 2 | 0/18 |

NumPy-owned `from_numpy`, Torch-owned `clone`, and runtime-like owned tensors
all give the same outputs within each profile. All values and contiguous
strides agree; changing pointer alignment to 64 bytes does not repair the
unbounded-BLAS mismatch. A separate same-process causal cross-over retains
the exact model, input pointers, strides, Torch threads and MKLDNN setting,
changing only BLAS threads **24 → 2 → 24**: mismatch → exact → original mismatch
for each of the first three inputs of both arms. This isolates BLAS execution
width rather than archive corruption or tensor storage ownership.

Small absolute differences are not necessarily a few ULPs near zero. In the
first three W0 inputs, the unbounded-BLAS maximum XY absolute error is
3.814697265625e-6, but maximum XY ULP distance is 15,232. No tolerance was
introduced or fitted. The original run had exact archived inputs but not
bitwise-equivalent execution settings; the new run verifies bitwise output
replay for all 366 archived forwards under matched settings. This is not a
new simulator/control trial or a safety approval.

Receipts: [profile index](cpu_replay_diagnosis_v1/index.json),
[complete exact replay](cpu_replay_diagnosis_v1/live2.json),
[BLAS-only causal cross-over](cpu_replay_diagnosis_v1/blas_crossover.json).
Complete replay receipt SHA256:
`654564953bd656f7aa9ed711f8969b2afa7f66143b30088961d924730efe8eae`.
Six focused tests pass. CPU only; zero training or simulator steps.

Repeat the unchanged original validator with its existing pinned arguments
and a **new** output path, prefixing the command with:

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/replay_exact_warmup_inputs_20261001.py ... --output NEW_RECEIPT.json
```

Set the variables before Python imports NumPy/Torch. Future CPU receipts
should report native BLAS pool width in addition to `torch.get_num_threads()`.
The diagnostic sources are `diagnose_warmup_cpu_replay_20261001.py` and
`diagnose_warmup_blas_crossover_20261001.py`; neither edits the original validator.
