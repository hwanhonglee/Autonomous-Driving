# PC2 v1.1 complete flattened source snapshot

Date: 2026-08-11 KST

The v1.1 branch and `IONIQ_EV_308_PC2_a` tag are published as one parentless root snapshot. The
historical v1.0 commit is recorded below for provenance only; it is not a parent of the new release.

This report defines what "all packages, with no submodule omission" means for the PC2 v1.1
release. It supplements the functional release and test reports; it does not expand their runtime
PASS boundary.

## Source roots and merge rule

The release worktrees were prepared from the historical PC2 v1.0 branches and overlaid with the
current PC2 working trees:

| Workspace | Current PC2 source | Release worktree |
|---|---|---|
| Autoware | `/home/a/autoware` | `/home/a/Autonomous-Driving-PC2-v1.1-autoware` |
| Driver | `/home/a/ros2_ws` | `/home/a/Autonomous-Driving-PC2-v1.1-ros2_ws` |

The merge is a union, not a destructive mirror. All current editable PC2 packages are present,
while the historical branch's 31 `src/tools` packages are retained. Files that already received a
reviewed v1.1 fix remain at the tested v1.1 revision rather than being overwritten by an older live
copy.

## Git/submodule result

The release branches contain:

- zero `.gitmodules` files;
- zero Git entries with mode `160000` (gitlinks/submodules);
- zero nested `.git` directories or files;
- zero tracked symlinks; the former YOLOX launch alias is stored as an ordinary XML blob.

Four calibration-document assets inherited as Git-LFS pointer text were resolved from the primary
CalibrationTools upstream and committed as ordinary PNG/PDF blobs. Each downloaded file's SHA-256
matches the `oid sha256` recorded by its former pointer, so a normal clone requires neither Git LFS
nor a second fetch to obtain those assets.

The `.git` file at each release worktree root is a normal Git worktree pointer and must not be
deleted. It is repository control metadata, not a submodule.

Three source trees were nested Git repositories on the live machine: Eagleye, topic_tools, and the
Lucid driver. Their working-tree contents are stored in the release as ordinary files; their nested
`.git` metadata is not copied. Consequently GitHub receives the actual package contents rather than
a gitlink or an empty directory.

## Package completeness

The current Autoware tree has 350 editable source packages when generated build/install and an
extracted deb copy are excluded. The release contains every one of those package paths, plus the 31
historical `src/tools` packages retained from the base branch:

| Set | Package manifests |
|---|---:|
| Current normal Autoware packages | 348 |
| Flattened topic_tools packages | 2 |
| Historical branch tools retained | 31 |
| Final Autoware release total | 381 |
| Missing current editable package paths | 0 |

The exact 381 paths are recorded in
`src/migration_work/inventory/PC2_V1.1_PACKAGE_PATHS.txt`.

The flattened topic_tools source is official tag `1.1.2`, upstream commit
`0fc927b7c0af0aaffae34a947b6ea4a7f9f97c94`, with the accepted relay shutdown patch applied. Its
source manifest is
`src/migration_work/vendor-src/topic_tools-1.1.2/VENDORED_SOURCE_MANIFEST.sha256`. The overlay
builder accepts either a verified upstream Git clone or this hash-verified flattened source tree.

The companion ros2 workspace contains the complete current `lucid_vision_driver` working tree as
ordinary files. Its inactive historical/test camera profiles are retained for provenance but are
not selected by `run_autoware`; the active contract remains `param/loop_top_cam.yaml` with the
serial-214 calibration.

## Current PC2 configuration overlay

In addition to the previously committed active v1.1 fixes, the release preserves current local
map, sensor-kit, camera/TLR, detector, RViz, and launch configuration files, including local
`copy_org` reference files. These files are included to avoid data loss; inclusion is not evidence
that every inactive or historical profile has been runtime-tested.

Several preserved package defaults are not the active PC2 contract. In particular, the top-level
PC2 launch explicitly supplies the generic YOLOX `rois0` output and normalized camera topics, while
some package-local launch defaults retain older debug/camera topic names. PC2 also launches neither
map nor localization, so the preserved C-track map-loader defaults and their metadata semantics are
not covered by the PC2 runtime PASS. Do not infer a PC3 map configuration PASS from their presence.

The complete-source review also corrected three packaging hazards without discarding their history:

- restored the historical traffic-light calibration file to its internally consistent 1920x1080
  dimensions (the imported 640x480 dimensions did not match its principal point/intrinsics);
- removed a duplicate `pointcloud_map_path` declaration from the pointcloud map-loader launch;
- moved the permissive old `autoware.launch copy.xml` out of the installable launch directory and
  into `migration_work/reference_snapshots`, preventing accidental PC2 planning/control startup.

The authoritative active-path and acceptance evidence remains:

- `PC2_V1.1_RELEASE_NOTES.md`;
- `PC2_V1.1_TEST_REPORT.md`;
- the guarded `run_pc2_autoware.sh` entry point;
- the serial-214 Lucid profile in the companion driver branch.

## Deliberate non-source exclusions

The following are not editable source packages and are intentionally not committed:

- any nested `.git` metadata;
- root or package `build`, `install`, and `log` output trees;
- `migration_work/vendor-build`, which includes machine-specific build products and absolute
  symlinks;
- `migration_work/vendor/ros-humble-topic-tools_1.1.2`, an extracted binary deb duplicate of the
  committed topic_tools source;
- Python caches, core dumps, temporary images, rosbag data, and bulk runtime logs;
- `/home/a/ros2_ws/src.zip`, an obsolete archive containing stale Git metadata.

These exclusions do not remove a unique editable ROS package. Build and runtime evidence that is
small, curated, and referenced by the release reports remains committed.

## Static validation result

- all XML files added or changed by the complete-source snapshot pass `xmllint`;
- all Python source in the materialized tree passes AST parsing;
- changed YAML parameter files parse with PyYAML;
- the flattened topic_tools SHA-256 manifest and shell syntax checks pass;
- current editable package paths: 350; missing from release: 0;
- remaining Git-LFS pointer files: 0.

A whole-tree XML scan also exposes three unchanged historical Tamagawa IMU launch files with
malformed node tags: `serial_AU7554N.launch.xml`, `serial_TAG250_Nxx40.launch.xml`, and
`serial_TAG264.launch.xml`. They predate this snapshot, are not in the PC2 camera/perception launch
path, and were not silently repaired as part of source packaging. They remain a known baseline
defect if that driver is later activated.

## Required commit-time checks

The final release is accepted only when all of the following hold on both worktrees:

```text
.gitmodules count                     = 0
gitlink (mode 160000) count           = 0
nested .git count                     = 0
missing current source package paths  = 0
Git objects >= 100 MiB                = 0
git diff --check errors               = 0
```

Push only the two explicit v1.1 branch refs and the two explicit IONIQ EV 308 tags. Do not use
`git push --mirror` or `git push --tags`: an unrelated legacy reachable-history tag contains an old
gitlink and is outside this release.
