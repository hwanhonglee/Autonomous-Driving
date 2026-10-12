# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Exercise the exact isolated agent class without a live simulator."""
import ast
import io
import json
import math
import pytest
from pathlib import Path
from types import SimpleNamespace as NS
from scripts.e2e.research_stop_preview import bounded_normal_step


@pytest.mark.parametrize('hazard', [False, True])
@pytest.mark.parametrize('worker', ['collect_research_curve_reference.py', 'collect_research_stop_preview.py'])
def test_agent_caps_speed_without_editing_emergency_control(hazard, worker):
    """HH_260906 - Match CARLA's no-debug run_step signature and preserve its returned brake."""
    path=next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())/'tools/e2e'/worker
    tree=ast.parse(path.read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='CurveReferenceAgent')
    control=NS(throttle=0.,brake=.5,steer=.1)
    class Agent:
        def __init__(self,vehicle,target_speed,**kwargs):
            self.speed=target_speed
            self.planner=NS(run_step=lambda:control)
        def get_local_planner(self): return self.planner
        def set_target_speed(self,speed): self.speed=speed
        def _affected_by_traffic_light(self): return hazard, NS(id=42) if hazard else None
        def _vehicle_obstacle_detected(self): return False, None, -1
        def add_emergency_stop(self, value):
            value.brake=.5
            return value
        def run_step(self):
            self._vehicle_obstacle_detected()
            affected, _ = self._affected_by_traffic_light()
            normal=self.planner.run_step()
            return self.add_emergency_stop(normal) if affected else normal
    stream=io.StringIO()
    vehicle=NS(get_transform=lambda:NS(location=NS(x=2.,y=3.),rotation=NS(yaw=0.)),
        get_velocity=lambda:NS(x=1.,y=0.,z=0.),get_world=lambda:NS(get_snapshot=lambda:NS(frame=12)),
        get_traffic_light_state=lambda:'Red',is_at_traffic_light=lambda:True)
    env=dict(original=Agent,math=math,json=json,args=NS(wheelbase_m=2.85,physics_hz=20.),stream=stream,
        signal_preview=lambda *args:None,bounded_normal_step=bounded_normal_step,
        reference=NS(local_speed=lambda _:4.),
        projector=NS(length_m=100.,project=lambda *args:NS(progress_m=2.,cross_track_error_m=.1)))
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),env)
    agent=env['CurveReferenceAgent'](vehicle,target_speed=30)
    assert agent.run_step() is control
    if hazard: assert control.brake==.5
    elif worker=='collect_research_stop_preview.py': assert control.brake<=.04
    assert agent.speed==14.4
    agent.set_target_speed(7.)
    assert agent.run_step() is control
    assert agent.speed==7.
    assert all(json.loads(line)['training_data_approved'] is False for line in stream.getvalue().splitlines())
    records=[json.loads(line) for line in stream.getvalue().splitlines()]
    assert all(r['upstream_hazard_decisions']['emergency_override_called'] is hazard for r in records)
    assert all(r['upstream_hazard_decisions']['traffic_light_actor_id']==(42 if hazard else None) for r in records)
