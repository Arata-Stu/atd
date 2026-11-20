"""Trackside geometry, optical visibility and provenance of the southern straight."""
from math import cos, dist, sin
from collections import defaultdict
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import monza_ascari
import monza_trackside
import scenery


class AscariSceneryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, cls.route = geography.load_track('lap')
        cls.data = scenery.monza_features(True)
        cls.reference = scenery.Features([monza_ascari.REFERENCE], cls.data.project)
        cls.meshes = {name: faces for name, _, faces in monza_ascari.details(cls.edges, cls.data)}

    def test_equipment_clears_racing_surface_gravel_and_grandstands(self):
        road = [self.edges['left'], self.edges['right']]
        exclusions = [rings for kind in ('grandstand', 'sand', 'building')
                      for _, rings in self.data.polygons(kind)]
        # Limit the expensive point tests to XY bounding boxes near each asset.
        polys = [(min(x for x, _ in rings[0]), max(x for x, _ in rings[0]),
                  min(y for _, y in rings[0]), max(y for _, y in rings[0]), rings)
                 for rings in exclusions]
        for name, faces in self.meshes.items():
            xy = {(x,y) for face in faces for x,y,_ in face}
            for x,y in xy:
                self.assertFalse(scenery._inside(x,y,road), name)
                self.assertFalse(any(scenery._inside(x,y,rings) for x0,x1,y0,y1,rings in polys
                                     if x0 <= x <= x1 and y0 <= y <= y1), name)

    def test_four_flights_land_on_the_existing_bridge_ends(self):
        rails = scenery.Features([monza_trackside.REFERENCE], self.data.project)
        ends = set(pt for props, line in rails.lines('overpass') if props['tags']['highway']=='footway'
                   for pt in line)
        stairs = list(self.reference.lines('steps'))
        self.assertEqual({top for _, (_, top) in stairs}, ends)
        self.assertEqual(sorted(int(props['tags']['step_count']) for props, _ in stairs
                                if 'step_count' in props['tags']), [27, 30])
        for props, (bottom, top) in stairs:
            count = int(props['tags'].get('step_count', round(dist(bottom,top)/.025)))
            faces = []
            monza_ascari._stairs(faces, bottom, top, count)
            # A horizontal tread at each successive elevation reaches the deck at 0.60 m.
            levels = {round(face[0][2],5) for face in faces if len({v[2] for v in face}) == 1}
            self.assertTrue(all(round(.60*i/count,5) in levels for i in range(1,count+1)))

    def test_fan_zone_stand_faces_track_between_bridges_and_clears_trees(self):
        (stand, (ring,)), = self.reference.polygons('grandstand')
        self.assertEqual(stand['tags']['name'], 'Rettilineo Parabolica Esterna')
        self.assertIn(stand, [p for p, _ in self.data.polygons('grandstand')])
        # On the left-hand verge with the front row towards the road.
        setbacks = []
        for x, y in ring:
            station, px, py, yaw = min(self.route, key=lambda p: dist((x, y), p[1:3]))
            self.assertLess(416, station)
            self.assertLess(station, 428)
            setbacks.append(-(x - px) * sin(yaw) + (y - py) * cos(yaw))
        self.assertLess(2., min(setbacks))
        self.assertLess(max(setbacks[:2]), min(setbacks[2:]))
        for _, trees in scenery.woodland(data=self.data):
            for face in trees:
                a, b = face[0][:2], face[1][:2]
                self.assertFalse(scenery._inside(*a, [ring]) or scenery._inside(*b, [ring]))
                self.assertFalse(scenery._crosses_ring(a, b, ring))

    def test_signs_face_approaching_driver_on_both_verges(self):
        boards = self.reference.by_kind['distance_board']
        alboreto = [f for f in boards if f['properties'].get('turn') != 'T8']
        ascari = [f['properties'] for f in boards if f['properties'].get('turn') == 'T8']
        self.assertEqual([(f['side'], f['label']) for f in ascari], [('right', '100')])
        self.assertEqual({(f['properties']['side'], f['properties']['label']) for f in alboreto},
                         {(side, value) for side in ('left','right') for value in ('150','100','50')})
        for f in boards:
            p = f['properties']
            s,x,y,yaw = min(self.route, key=lambda point: abs(point[0]-p['station_rc_m']+4))
            cx,cy = self.data.project(*f['geometry']['coordinates'])
            dx,dy = cx-x,cy-y
            depth, lateral = dx*cos(yaw)+dy*sin(yaw), dy*cos(yaw)-dx*sin(yaw)
            self.assertEqual(lateral>0, p['side']=='left')
            for eye in (-.0475,.0475):
                self.assertLess(abs(lateral-eye), (depth-.18)*.94)
            self.assertGreater(cos(yaw-p['heading_rad']), .99)
        self.assertEqual(len(self.meshes['ascari_panel_face']),3)

    def test_rail_splices_follow_alignment_and_face_the_driver(self):
        for side, sign in [('left', -1), ('right', 1)]:
            batches = defaultdict(list)
            # Off-grid vertex must not reset the 0.40 m pitch or clip a joint.
            monza_ascari._rail_splices(batches, [(0,0),(.39,0),(.805,0)], side)
            faces = batches['lap_steel']
            self.assertEqual(len(faces), 2*2*4)
            self.assertEqual({round(x,4) for face in faces for x,_,_ in face},
                             {.3684,.4,.7684,.8})
            self.assertTrue(all(y*sign>0 for face in faces for _,y,_ in face))
            self.assertEqual(len(batches['bolt_heads']), 16)

    def test_new_left_catch_fence_follows_the_segmented_rail(self):
        barriers = scenery.Features([monza_trackside.REFERENCE], self.data.project)
        spans = [line for props,line in barriers.lines('guard_rail')
                 if props.get('side')=='left' and props.get('sector_rc_m')==[395,484]]
        self.assertEqual(len(spans), 4)
        right_spans = [line for props,line in barriers.lines('fence') if props.get('side')=='right'
                       and props.get('sector_rc_m') in ([462,476], [480,485])]
        mesh = self.meshes['ascari_catch_mesh']
        self.assertEqual(len(mesh), sum(2*(len(s)-1) for s in spans)
                         + sum(len(s)-1 for s in right_spans))
        for sp in (402, 420, 430, 450, 470):
            _,x,y,yaw = min(self.route,key=lambda pt:abs(pt[0]-sp))
            points = [(vx,vy) for face in mesh for vx,vy,_ in face]
            nearby = [(px-x,py-y) for px,py in points if dist((px,py),(x,y)) < 5]
            self.assertTrue(nearby)
            sides = {dy*cos(yaw)-dx*sin(yaw)>0 for dx,dy in nearby}
            self.assertEqual(sides, {True,False} if sp==470 else {True})


if __name__ == '__main__':
    unittest.main()
