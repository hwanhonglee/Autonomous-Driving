"""HH_261004 - Prepare V1 navigation request law with a separately declared V2 semantic mesh.

No controller law, observed state, original physical fields, witness or safety
threshold changes.  This new revision replaces only the terminal decoder grid;
ordinary calls still delegate exactly.  Frozen V1 remains unchanged and all
failed V1 evidence remains part of the disclosed comparison denominator.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

from .navigation_terminal_retiming_v1_20261004 import NavigationTerminalRetimingV1, _require
from .terminaldecoder_v2_20261004 import PathTimeDecoderTerminalV2


SCHEMA = "portable_e2e.navigation_terminal_retiming.prepared.20261004.v2"
FROZEN_NAVIGATION_V1_SHA256 = "99c90510a3bd53440d676cf0dedd8b1f3e7ea31bea1551784ef957cc77986690"
TERMINAL_V2_SHA256 = "596e247bc6c5f5574d5bbba0fc337d82111080a62c3e937d0f147c031d79c23e"


def verify_revision_source_closure():
    # HH_261004 - Reject aliases/nonregular code sources and bind the frozen law and newly declared mesh separately.
    pins = {"navigation_terminal_retiming_v1_20261004.py": FROZEN_NAVIGATION_V1_SHA256,
            "terminaldecoder_v2_20261004.py": TERMINAL_V2_SHA256}
    parent = Path(__file__).absolute().parent
    actual = {}
    for name in pins:
        path = parent / name
        _require(path.resolve(strict=True) == path and path.is_file(),
                 "NAVIGATION_V2_SOURCE_ALIAS", "revision code source must be canonical regular file")
        actual[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    _require(actual == pins, "NAVIGATION_V2_SOURCE_MISMATCH", "frozen request law or declared semantic mesh differs")
    return actual


class NavigationTerminalRetimingV2(NavigationTerminalRetimingV1):
    """HH_261004 - Keep the entire frozen forward request law; replace terminal grid only."""

    def __init__(self, config=None):
        self.revision_source_sha256 = verify_revision_source_closure()
        super().__init__(config)
        self.decoder = PathTimeDecoderTerminalV2(config)


def prepared_contract():
    """HH_261004 - Distinguish controller/numerical intervention from optimizer or capability approval."""
    return dict(schema=SCHEMA, prepared_cpu_only=True, prepared_not_deployed=True,
        revision_source_sha256=verify_revision_source_closure(),
        frozen_navigation_v1_sha256=FROZEN_NAVIGATION_V1_SHA256,
        request_law_forward_inherited_unchanged=True, ordinary_outputs_errors_exact_delegate=True,
        terminal_change="V2 semantic grid only; all V1 physical envelope/audit/time rules retained",
        observed_state_clipped=False, actual_support_changed=False, raw_semantic_heads_used=False,
        model_or_weights_changed=False, learned_STOP_claim=False, vehicle_control_approved=False,
        main_functions_approved=0, subfunctions_approved=0)
