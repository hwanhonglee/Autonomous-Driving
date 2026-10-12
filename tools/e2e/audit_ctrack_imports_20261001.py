#!/usr/bin/env python3
"""HH_261001 - Decode the pinned uncooked UE4.26 CTrack map's direct imports.

Supports only this map's observed UE4 version 522, legacy version -7, and
unfiltered editor format. Serialization is grounded in the installed engine's
PackageFileSummary.cpp, ObjectResource.cpp and UnrealNames.cpp. This proves
direct package file availability, not transitive load success or pixel labels.
"""

from __future__ import annotations

from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path
import struct

try:
    from . import prepare_ctrack_isolated_probe_20261001 as stage
except ImportError:
    import prepare_ctrack_isolated_probe_20261001 as stage


class Reader:
    def __init__(self, data: bytes):
        self.data, self.offset = data, 0

    def integer(self) -> int:
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def string(self) -> str:
        length = self.integer()
        if abs(length) > len(self.data):
            raise ValueError("FString length exceeds package size")
        count = abs(length) * (2 if length < 0 else 1)
        raw = self.data[self.offset:self.offset + count]
        if len(raw) != count:
            raise ValueError("truncated FString")
        self.offset += count
        return raw.decode("utf-16le" if length < 0 else "utf-8").rstrip("\0")

    def name(self, names: list[str]) -> str:
        index, number = self.integer(), self.integer()
        if not 0 <= index < len(names) or number < 0:
            raise ValueError("invalid serialized FName")
        # UE FName uses zero for no suffix, external suffix+1 otherwise.
        return names[index] + (f"_{number - 1}" if number else "")


def decode_imports(data: bytes) -> tuple[dict, list[dict]]:
    reader = Reader(data)
    magic, legacy, ue3, version, licensee = (reader.integer() for _ in range(5))
    if (magic & 0xFFFFFFFF, legacy, ue3, version, licensee) != (0x9E2A83C1, -7, 864, 522, 0):
        raise ValueError("unsupported package version; require the pinned UE4.26 editor map")
    custom_count = reader.integer()
    if not 0 <= custom_count <= 100:
        raise ValueError("invalid optimized custom-version count")
    reader.offset += custom_count * 20
    header_size, folder = reader.integer(), reader.string()
    flags = reader.integer() & 0xFFFFFFFF
    if flags != 0x20000:
        raise ValueError("unexpected editor map package flags")
    name_count, name_offset = reader.integer(), reader.integer()
    localization_id = reader.string()
    gather_count, gather_offset, export_count, export_offset, import_count, import_offset, depends_offset = (
        reader.integer() for _ in range(7))
    if (min(name_count, import_count, export_count) <= 0 or
            not 0 < name_offset < import_offset < export_offset < depends_offset <= header_size <= len(data) or
            import_offset + import_count * 36 != export_offset):
        raise ValueError("package table offsets or import record stride disagree")
    reader.offset = name_offset
    names = []
    for _ in range(name_count):
        names.append(reader.string())
        reader.offset += 4  # Two serialized uint16 name hashes, since UE4 version 504.
    if reader.offset > import_offset:
        raise ValueError("name table overlaps the import table")
    reader.offset = import_offset
    records = []
    for index in range(import_count):
        records.append({"index": index + 1, "class_package": reader.name(names),
                        "class_name": reader.name(names), "outer_index": reader.integer(),
                        "object_name": reader.name(names), "package_name": reader.name(names)})
    if reader.offset != export_offset:
        raise ValueError("import table end does not match export table start")
    summary = {"version": version, "legacy": legacy, "custom_version_count": custom_count,
               "header_size": header_size, "flags": flags, "folder": folder,
               "name_count": name_count, "name_offset": name_offset,
               "import_count": import_count, "import_offset": import_offset,
               "export_count": export_count, "export_offset": export_offset,
               "depends_offset": depends_offset, "import_record_bytes": 36,
               "localization_id": localization_id,
               "gatherable_text_count": gather_count, "gatherable_text_offset": gather_offset}
    return summary, records


def audit() -> dict:
    payload = stage.LEVEL.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != stage.PINNED[stage.LEVEL]:
        raise ValueError("staging map SHA256 changed")
    summary, imports = decode_imports(payload)
    packages = []
    for record in imports:
        package = record["object_name"]
        if record["class_name"] == "Package" and package.startswith(("/Game/", "/Engine/")):
            path = stage.resolve_package(package)
            packages.append({"import_index": record["index"], "package": package,
                             "path": str(path) if path else None,
                             "size_bytes": path.stat().st_size if path else None})
    packages_by_index = {r["import_index"]: r for r in packages}
    semantic_meshes = []
    for record in imports:
        if record["class_name"] == "StaticMesh" and record["outer_index"] < 0:
            package = packages_by_index.get(-record["outer_index"])
            if package and package["package"].startswith("/Game/map_package/Static/"):
                folder = package["package"].split("/")[4]
                if folder in ("Road", "RoadLine", "SideWalk"):
                    semantic_meshes.append({"import_index": record["index"],
                                            "object_name": record["object_name"],
                                            "package": package["package"], "tagger_folder": folder})
    unknown = [p for p in packages if p["path"] is None]
    imported_names = {p["package"] for p in packages}
    suspicious = "/Game/map_package/Maps/C_track_1_0_7/C_track_1_0"
    return {"schema": "portable_e2e.ctrack_direct_import_audit.v1",
            "status": "PASS" if not unknown else "FAIL", "level": str(stage.LEVEL),
            "level_sha256": digest, "package_summary": summary,
            "direct_content_package_imports": packages, "missing_direct_content_packages": unknown,
            "semantic_static_mesh_imports": semantic_meshes,
            "semantic_static_mesh_counts": dict(Counter(r["tagger_folder"] for r in semantic_meshes)),
            "old_printable_candidate": {"name": suspicious, "is_imported_package": suspicious in imported_names},
            "source_serializers": {str(path): stage.sha256(path) for path in (
                stage.ENGINE / "Source/Runtime/CoreUObject/Private/UObject/PackageFileSummary.cpp",
                stage.ENGINE / "Source/Runtime/CoreUObject/Private/UObject/ObjectResource.cpp",
                stage.ENGINE / "Source/Runtime/Core/Private/UObject/UnrealNames.cpp")},
            "transitive_dependencies_validated": False, "simulator_launched": False,
            "dataset_admitted": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        stage.write_json_new(args.output, result)
    print(json.dumps({"status": result["status"], "level_sha256": result["level_sha256"],
                      "direct_content_package_imports": len(result["direct_content_package_imports"]),
                      "missing_direct_content_packages": result["missing_direct_content_packages"],
                      "semantic_static_mesh_counts": result["semantic_static_mesh_counts"],
                      "old_printable_candidate": result["old_printable_candidate"],
                      "simulator_launched": False}, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
