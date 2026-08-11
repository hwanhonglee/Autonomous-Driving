# 12 — PC2 perception and camera

Evidence: `historical-source-derived` plus `runtime-observed-from-PC1-on-domain10`

Historical PC2 launches sensing, perception, and RViz. Its `ros2_ws` contains a Lucid
`ArenaCameraNode` for a top traffic-light camera, historically serial `212401044`, 20 fps. This
does not prove that device or serial belongs to the current vehicle.

Target PC2 owns perception/GPU inference and may own a camera driver only if the camera is
physically attached. It must not duplicate PC3 common sensing, map, localization, system, planning,
control, API, or RViz. In real mode it must be the single official predicted-object publisher; in
virtual mode that publisher must be suppressed in favor of PC4.

In the latest simultaneous Domain 10 window,
`/perception/object_recognition/objects` had one publisher,
`/perception/object_recognition/prediction/map_based_prediction`, at approximately 9.09 Hz.
Observed messages used frame `map` and contained zero objects. Transport and single-publisher
behavior passed for that window, but an empty scene does not validate detection quality or camera
hardware. No PC2 file was edited from PC1.
