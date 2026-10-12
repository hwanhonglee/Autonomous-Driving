"""Command-line entry point for the portable common10 dataset validator."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Sequence

from .contract import ContractError
from .contract import REPORT_SCHEMA_ID
from .contract import validate_dataset


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="directory containing dataset.json")
    parser.add_argument("--contract", type=Path, help="alternate common10 contract")
    parser.add_argument(
        "--mode",
        choices=("planning", "runtime", "schema"),
        default="planning",
        help="planning enforces 10 Hz gates; runtime also requires offline 1 ms readiness",
    )
    parser.add_argument(
        "--skip-image-sha256",
        action="store_true",
        help=(
            "skip declared SHA-256 comparison only; JPEG bytes are still read and "
            "parsed, and image integrity is reported NOT_RUN"
        ),
    )
    parser.add_argument("--output", type=Path, help="optional new JSON report path")
    parser.add_argument(
        "--replace-output",
        action="store_true",
        help="atomically replace an existing --output report",
    )
    return parser.parse_args(argv)


def _write_report(path: Path, report: dict, *, replace: bool) -> None:
    output = path.expanduser().resolve()
    if output.exists() and not replace:
        raise ContractError(f"output already exists; use --replace-output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(report, indent=2, allow_nan=False) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.tmp.", dir=output.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if replace:
            os.replace(temporary, output)
        else:
            try:
                os.link(temporary, output)
            except FileExistsError as error:
                raise ContractError(
                    f"output appeared while writing; refusing to replace it: {output}"
                ) from error
    finally:
        if temporary.exists():
            temporary.unlink()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        report = validate_dataset(
            args.dataset,
            contract_path=args.contract,
            mode=args.mode,
            check_image_hashes=not args.skip_image_sha256,
        )
        if args.output is not None:
            _write_report(args.output, report, replace=args.replace_output)
    except (ContractError, OSError, ValueError) as error:
        failure = {
            "schema_id": REPORT_SCHEMA_ID,
            "schema_version": 1,
            "status": "FAIL",
            "error": str(error),
        }
        print(json.dumps(failure, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
