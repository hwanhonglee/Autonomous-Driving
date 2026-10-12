#!/usr/bin/env python3
"""HH_261001 - Capture only the prospective Town01 p02/p04 same-tick RGB pairs."""

from contextlib import contextmanager
from pathlib import Path

from . import collect_map_paired_rgb_20261001_v2 as base


ROOT = Path(__file__).resolve().parents[2]
ROLE = "town01_p02_p04_paired_appearance_pending_qa"
ROUTES = {
    "788665af9e3b5449634f1c5392e94b2a6258ae3262d26e69f3cf81b96791489a": (
        "right", "town01_right_s0000_p02.json"
    ),
    "f4d474eee01892e16b8b78a69de5a3b32ccc8e4f227e30a9a89dc023dbfc25a7": (
        "left", "town01_left_s0000_p04.json"
    ),
}
ROUTE_ROOT = (
    ROOT
    / "storage/experiments/validation/2026-09-01/"
    "autoware_vad_town_matrix_30kph_v16_owned_window_final/"
    "maps/town01/catalog/turn/routes/town01"
)


def checked_route(path: Path) -> tuple[dict, str]:
    """HH_261001 - Bind both route content and its original catalog location."""
    import json

    if path.is_symlink() or not path.is_file():
        raise ValueError("p02/p04 route must be a regular catalog file")
    sha = base.digest(path)
    if sha not in ROUTES:
        raise ValueError("only exact p02/p04 route bytes are permitted")
    arm, filename = ROUTES[sha]
    expected = ROUTE_ROOT / arm / filename
    if path.absolute() != expected or path.resolve(strict=True) != expected:
        raise ValueError("p02/p04 route must be the pinned catalog path")
    route = json.loads(path.read_text())
    if (
        route.get("town") != "Town01"
        or route.get("scenario") != arm
        or route.get("weather") != "ClearNoon"
        or route.get("coordinate_reference") != "base_link"
        or len(route.get("route", [])) < 2
    ):
        raise ValueError("Town01 p02/p04 route metadata differs")
    return route, sha


@contextmanager
def scoped_parent():
    """HH_261001 - Reuse the frozen sensor engine under an isolated route scope."""
    old_routes, old_role = base.ROUTES, base.ROLE
    base.ROUTES = {sha: ("Town01", arm) for sha, (arm, _) in ROUTES.items()}
    base.ROLE = ROLE
    try:
        yield
    finally:
        base.ROUTES, base.ROLE = old_routes, old_role


def parse_args(argv=None):
    with scoped_parent():
        args = base.parse_args(argv)
    checked_route(args.route_file)
    if args.frames != 240 or args.allow_map_load or args.motion_mode != "basic-agent":
        raise ValueError("p02/p04 capture must use 240 physical BasicAgent ticks")
    return args


def run(args):
    import json

    route, sha = checked_route(args.route_file)
    with scoped_parent():
        base.run(args)
    # HH_261001 - Mark the newly captured frames as pending, regardless of older FALSE admission.
    summary_path = args.output / "summary.json"
    summary = json.loads(summary_path.read_text())
    summary.update(
        capture_study="town01_p02_p04_same_tick_20261001_v1",
        capture_site="p02" if route["scenario"] == "right" else "p04",
        route_sha256=sha,
        qa_status="PENDING_QA",
        dataset_admitted=False,
        train_eligible=False,
        independent_dev_approved=False,
        route_scope="new 240-frame physical run on existing Town01 p02/p04 route definition",
    )
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    run(parse_args())
