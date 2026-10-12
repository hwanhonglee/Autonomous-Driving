# Common measurement-only addendum before W0/W1 live tests

<!-- HH_261001 - Preserve actual numeric model inputs for replay; driving policy remains fixed. -->

Registered after training launch but before either W0/W1 live test or final model
evaluation. Apply the same observer to both arms. No model, input value, controller,
runtime threshold, route, initial handoff or trial duration changes.

Existing camera archives contain JPEG95, whereas live inference uses raw BGRA.
The first original-AH JPEG replay differs by up to0.00780m trajectory,
0.00273m/s speed and0.00933logit; exact replay cannot be claimed from JPEG alone.

Save all six actual model-input tensors returned by build_runtime_tensors for
every attempted forward, before the unchanged model call, as uncompressed
data-only NPZ. Bind each file and tensor hash to exact timestamps, checkpoint
identity and the recorded raw model-output hash in an append-only journal.
Inputs and outputs are never rewritten. Emergency steps that never build inputs
have no invented snapshot.

Input-copy time remains inside the existing100ms policy deadline. Archive writing
happens after the unchanged policy step and before control return, so its measured
wall time is separately reported; it may affect synchronous simulation wall speed.
Do not call the original inference timing end-to-end real-time latency. Both arms
use identical capture behavior; an archive error fails the capture rather than
silently disabling evidence. No new training-data or driving approval follows.
