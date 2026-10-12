"""HH_260906 - Check algebra and reject impossible future-oracle summaries."""
import pytest
from scripts.e2e.summarize_ad_selection_gap import summarize


def document(selected, oracle):
    return {"subgroups": {g: {"metrics": {"selected_ade_m": selected, "oracle_ade_m": oracle}}
                          for g in ("town03_right_turn_dev", "town03_straight_dev")}}


def test_decomposition():
    result = summarize(document(4, 1), {"evaluation": document(3.5, 1.2)})
    d = result["groups"]["town03_right_turn_dev"]["delta_ad_minus_k1"]
    assert d["selected_ade_m"] == pytest.approx(d["oracle_ade_m"] + d["selection_gap_m"])
    assert d["selection_gap_m"] == pytest.approx(-.7)


def test_invalid_oracle():
    with pytest.raises(ValueError):
        summarize(document(1, 2), {"evaluation": document(1, 1)})
