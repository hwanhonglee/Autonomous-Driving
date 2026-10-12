"""HH_260906 - Verify the inactive integration seam using mocks only, without CARLA imports."""
import pytest
from scripts.e2e.offline_turn_speed_envelope import plan
from scripts.e2e.research_speed_reference import SpeedReference,ReferenceRequest
from scripts.e2e.research_agent_reference_adapter import ResearchAgentReferenceAdapter


class Agent:
    """HH_260906 - Stand in for an upstream agent and expose any unintended pedal mutation."""
    def __init__(self):
        self.speeds=[];self.calls=0
        self.control=dict(throttle=.1,brake=.5,steer=.2,hand_brake=False)
    def set_target_speed(self,value):
        """HH_260906 - Record proposed km/h without applying vehicle actuation."""
        self.speeds.append(value)
    def run_step(self):
        """HH_260906 - Return the exact sentinel control object."""
        self.calls+=1;return self.control


@pytest.fixture
def adapter():
    """HH_260906 - Instantiate only pure Python objects for the integration tests."""
    return ResearchAgentReferenceAdapter(Agent(),SpeedReference(plan([dict(distance_m=i,yaw=0.) for i in range(101)])))


def test_no_pedal_override(adapter):
    """HH_260906 - Preserve upstream braking even when an otherwise normal step discovers a hazard."""
    result=adapter.step(10,ReferenceRequest(40,4,.05))
    assert result['control'] is adapter.agent.control
    assert result['control']['brake']==.5
    assert adapter.agent.speeds==[pytest.approx(4.05*3.6)]


def test_emergency_keeps_original_target(adapter):
    """HH_260906 - Do not smooth or replace the upstream emergency path."""
    emergency=dict(throttle=0.,brake=1.)
    result=adapter.step(10,ReferenceRequest(40,8,.05,emergency=True),upstream_emergency_control=emergency)
    assert result['control'] is emergency and not adapter.agent.speeds and adapter.agent.calls==0


def test_missing_emergency_command_never_calls_normal_agent(adapter):
    """HH_260906 - An emergency flag alone is not proof that normal run_step will return braking."""
    result=adapter.step(10,ReferenceRequest(40,8,.05,emergency=True))
    assert result['control'] is None and result['status']=='OWNER_EMERGENCY_REQUIRED'
    assert adapter.agent.calls==0


def test_impossible_reference_requires_owner_failsafe(adapter):
    """HH_260906 - Refuse normal actuation when the requested stop is already infeasible."""
    result=adapter.step(10,ReferenceRequest(40,8,.05,stop_required=True,stop_distance_m=1))
    assert result['control'] is None and adapter.agent.calls==0


def test_upstream_stop_not_overridden(adapter):
    """HH_260906 - Never replace a lower goal or hazard target with a higher curve reference."""
    result=adapter.step(10,ReferenceRequest(40,4,.05),upstream_speed_cap_mps=0)
    assert result['status']=='UPSTREAM_STOP_REQUIRES_FAILSAFE' and result['control'] is None


def test_upstream_cap_unit_conversion(adapter):
    """HH_260906 - Keep the explicit m/s to km/h conversion consistent."""
    result=adapter.step(10,ReferenceRequest(40,4,.05),upstream_speed_cap_mps=4)
    assert result['requested_speed_kph']==14.4


@pytest.mark.parametrize('frame',[10,9,12])
def test_duplicate_backward_or_skipped_frame(adapter,frame):
    """HH_260906 - Refuse repeated or skipped synchronous observations."""
    adapter.step(10,ReferenceRequest(40,4,.05))
    with pytest.raises(ValueError):adapter.step(frame,ReferenceRequest(40,4,.05))


def test_consecutive_step(adapter):
    """HH_260906 - Accept a correctly advancing explicit frame sequence."""
    adapter.step(10,ReferenceRequest(40,4,.05))
    assert adapter.step(11,ReferenceRequest(40.2,4.05,.05))['control'] is adapter.agent.control
