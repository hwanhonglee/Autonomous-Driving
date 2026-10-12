"""HH_260906 - Keep terminal alignment explicit and reject nonstraight approaches."""
import numpy as np
import pytest
from scripts.e2e.prepare_terminal_alignment_ab import align_terminal


def test_goal_change_disclosed_and_input_preserved():
    points=[[float(x),0.] for x in range(15)];points[-1][1]=.3
    result,report=align_terminal(points)
    assert result[-1][1]==pytest.approx(0.)
    assert points[-1][1]==.3
    assert report['goal_changed'] and report['goal_shift_m']==pytest.approx(.3)


def test_curve_not_replaced_with_straight():
    with pytest.raises(ValueError):
        align_terminal([[float(x),float(np.sin(x))] for x in range(15)])
