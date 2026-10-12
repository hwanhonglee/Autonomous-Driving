# Fixed native-panel review

<!-- HH_261001 - Qualitative review of preregistered indices, never frame selection or tuning. -->

Each comparison keeps original 640×360 RGB, GT and both predictions unscaled.
The independent recount checks every displayed panel pixel against the raw
arrays and saved predictions. These observations supplement all-frame metrics.

## CTrack LEFT: 000/119/239 reviewed

The photometric candidate removes many false sidewalk islands from the road.
At119 the yellow centerline is cleaner, with fewer unrelated fixture/pole
predictions along it; at239 the near road is more continuous. However, the
right white edge remains largely unrecognized as lane marking, the sidewalk
boundary is ragged, and tree/foliage fixture/pole false positives persist.
Frame000 still misses large near lane-marking segments. This is partial
semantic improvement, not complete road-boundary reconstruction.

## CTrack STRAIGHT: 000/119/239 reviewed

At000 both models classify most of the shadow-patterned road as sidewalk.
At119 the candidate opens a larger continuous road strip, but much of the
road remains purple. At239 both models do substantially better; the candidate
has a cleaner centerline and fewer foreground artifacts. White edge-line
recall remains weak at all three fixed positions; tree/pole/fixture false
positives persist. Reduced all-frame road-to-sidewalk confusion does not
amount to a solved straight-route domain shift.

## Paired Town01 right-p01: 000/119/239 reviewed, both RGB conditions

Road/sidewalk separation and yellow markings remain visually strong in both
conditions. At000 the candidate removes a large false red fixture patch on
the dark right facade in RGBfalse. At119/239, RGBtrue has severe retained
glare; both models still recover much of the road and centerline, while thin
pole/sign details are imperfect. These are fitted TRAIN examples. Their
strong appearance fit and small TRUE-condition road/lane losses cannot be
interpreted as new held-out robustness.

## Paired Town02 straight-p00: 000/119/239 reviewed, both RGB conditions

At000/239 both models handle large, sharp building-shadow transitions on the
TRUE road considerably better than they handle CTrack's shadowed straight
road. Thus the residual external failure is not evidence of generic inability
to segment any shadow. At119 road and yellow centerline remain strong under
retained glare, while thin poles and fixtures are noisy. At239 the candidate
introduces purple sidewalk speckles into the near-right TRUE road/branch;
there are local regressions despite aggregate gains in other classes.

All twelve fixed PNGs have now been reviewed (six CTrack four-panel images
and six paired-TRAIN eight-panel images). No frame was removed or substituted.

## Interpretation boundary

The controlled joint gray/brightness intervention improved external road and
centerline metrics, supporting an appearance contribution. It does not
identify grayscale versus brightness separately or rule out material,
texture and spatial-context differences. White edge-line recall and thin
object false positives remain failures. The strong TRUE TRAIN fit together
with poor external STRAIGHT performance points to a coverage/generalization
gap, not evidence that simply adding optimization steps will solve it.
