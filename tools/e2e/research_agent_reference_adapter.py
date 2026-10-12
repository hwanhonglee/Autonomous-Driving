"""HH_260906 - Test-only agent seam for speed requests; never enabled by the shared collector."""
from scripts.e2e.offline_turn_speed_envelope import number


class ResearchAgentReferenceAdapter:
    """HH_260906 - Preserve upstream controls and reject normal commands when reference inputs are unsafe."""
    def __init__(self,agent,reference):
        self.agent=agent;self.reference=reference;self.previous_frame=None

    def step(self,frame,observation,*,upstream_speed_cap_mps=None,upstream_emergency_control=None):
        """HH_260906 - Issue no pedal edits; a None control requires the owning caller's fail-safe path."""
        if type(frame) is not int or frame<0 or (self.previous_frame is not None and frame!=self.previous_frame+1):
            raise ValueError('one monotonically consecutive frame per adapter step required')
        response=self.reference.update(observation)
        if response['status']=='EMERGENCY_PASSTHROUGH':
            # HH_260906 - A malformed optional comfort cap must not suppress an explicitly supplied emergency.
            self.previous_frame=frame
            return dict(reference=response,control=upstream_emergency_control,speed_request_sent=False,
                        status=('UPSTREAM_EMERGENCY_PRESERVED' if upstream_emergency_control is not None
                                else 'OWNER_EMERGENCY_REQUIRED'))
        cap=None
        if upstream_speed_cap_mps is not None:
            cap=number(upstream_speed_cap_mps)
            if cap<0:raise ValueError('negative upstream speed cap')
        self.previous_frame=frame
        if not response['normal_reference_allowed']:
            return dict(reference=response,control=None,speed_request_sent=False,status='OWNER_FAILSAFE_REQUIRED')
        target=response['target_speed_mps']
        if cap is not None:
            target=min(target,cap)
            if target < max(0,observation.speed_mps-self.reference.config['deceleration_mps2']*observation.dt_s)-1e-8:
                return dict(reference=response,control=None,speed_request_sent=False,status='UPSTREAM_STOP_REQUIRES_FAILSAFE')
        self.agent.set_target_speed(target*3.6)
        control=self.agent.run_step()
        return dict(reference=response,control=control,speed_request_sent=True,
                    requested_speed_kph=target*3.6,status='UNCHANGED_UPSTREAM_CONTROL')
