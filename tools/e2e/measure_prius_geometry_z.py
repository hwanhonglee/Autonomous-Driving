"""HH_260906 - Measure an owned stationary Prius bounding box; never drive or load a world."""
import argparse
import hashlib
import json
from pathlib import Path


def parse_args(argv=None):
    """HH_260906 - Bind the measurement to the existing owner's local server and route map."""
    p=argparse.ArgumentParser(allow_abbrev=False)
    p.add_argument('output',type=Path)
    p.add_argument('route_file',type=Path)
    p.add_argument('--host',choices=['127.0.0.1'],default='127.0.0.1')
    p.add_argument('--port',type=int,default=2100)
    a=p.parse_args(argv);a.allow_map_load=False
    if a.output.exists() or not 1024<=a.port<=65533:p.error('fresh output and valid port required')
    return a


def main():
    """HH_260906 - Destroy only the actor created here, including on measurement failure."""
    import carla
    a=parse_args();route=json.loads(a.route_file.read_text())
    client=carla.Client(a.host,a.port);client.set_timeout(10.)
    world=client.get_world()
    if world.get_map().name.rsplit('/',1)[-1]!=route['town']:raise ValueError('wrong map')
    if list(world.get_actors().filter('vehicle.*')):raise ValueError('unexpected existing vehicles')
    blueprint=world.get_blueprint_library().find('vehicle.toyota.prius')
    blueprint.set_attribute('role_name','owned_geometry_z')
    actor=None
    try:
        for transform in world.get_map().get_spawn_points():
            actor=world.try_spawn_actor(blueprint,transform)
            if actor is not None:break
        if actor is None:raise RuntimeError('no safe spawn')
        actor.set_simulate_physics(False)
        box=actor.bounding_box
        def xyz(v):return dict(x=v.x,y=v.y,z=v.z)
        document=dict(vehicle_id=actor.type_id,actor_id=actor.id,
            server_version=client.get_server_version(),map=world.get_map().name,
            bounding_box=dict(location=xyz(box.location),extent=xyz(box.extent),
                rotation=dict(roll=box.rotation.roll,pitch=box.rotation.pitch,yaw=box.rotation.yaw)),
            capture_base_link_convention='historical collector: actor center minus wheelbase/2 along forward',
            historical_collector_wheelbase_m=2.85,physics_disabled=True,driven=False,
            historical_actor_asset_identity_proven=False,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        a.output.mkdir(parents=True,exist_ok=False)
        (a.output/'measurement.json').write_text(json.dumps(document,indent=2))
        print(json.dumps(document))
    finally:
        if actor is not None:actor.destroy()


if __name__=='__main__':main()
