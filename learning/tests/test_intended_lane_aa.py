"""HH_260906 - Ensure corridor selection cannot silently admit the adjacent lane."""
from types import SimpleNamespace as NS
from shapely.geometry import box
from scripts.e2e.audit_intended_lane_aa import lane_corridor


def test_selected_lane_excludes_neighbor():
    class World:
        def get_waypoint_xodr(self,road,lane,s):
            assert lane==-1
            return NS(section_id=0,lane_width=4.,transform=NS(location=NS(x=s,y=0.),rotation=NS(yaw=0.)))
    xml=b'<OpenDRIVE><road id="1" length="10"><lanes><laneSection s="0"/></lanes></road></OpenDRIVE>'
    corridor,ids=lane_corridor(World(),xml,[dict(road_id=1,section_id=0,lane_id=-1)],.05)
    assert ids==[(1,0,-1)]
    assert corridor.covers(box(2,-1,5,1))
    assert box(2,1,5,3).difference(corridor).area>2.9
