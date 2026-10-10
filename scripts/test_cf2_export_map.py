import json
from pathlib import Path
import tempfile
import unittest
from cf2_export_map import export_map
from cf2_live_map import LiveMap
from test_cf2_live_map import pose,ranges

class ExportTests(unittest.TestCase):
    def test_pgm_orientation_origin_and_unknown_are_preserved(self):
        snapshot={'resolution':.5,'width':4,'height':2,'origin':[2.,-1.],
                  'start':[3.,-.5],'session_id':'test','data':[-1,0,100,-1,100,-1,0,0]}
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'map.json'
            files=export_map(snapshot,source)
            self.assertEqual(files[0].read_bytes(),b'P5\n4 2\n255\n'+bytes([0,205,254,254,205,254,0,205]))
            self.assertIn('origin: [-1.0, -0.5, 0.0]',files[1].read_text())
            manifest=json.loads(files[-1].read_text())
            self.assertFalse(manifest['map_validated_for_drone1'])
            self.assertFalse(manifest['shared_frame_aligned'])

    def test_crop_matches_survey_bounds(self):
        s={'resolution':.05,'width':160,'height':160,'origin':[-4.,-4.],
           'start':[0.,0.],'area':[5.,3.],'data':[-1]*25600}
        with tempfile.TemporaryDirectory() as d:
            files=export_map(s,Path(d)/'map.json')
            self.assertTrue(files[0].read_bytes().startswith(b'P5\n100 60\n255\n'))
            self.assertIn('origin: [-2.5, -1.5, 0.0]',files[1].read_text())

    def test_visited_centre_is_free_without_revealing_unknown_disk(self):
        m=LiveMap(mapping_radius=.8,record_visited_centres=True)
        m.update(pose(),now=1);m.update(ranges(front=32766),now=1)
        s=m.snapshot(1)
        self.assertEqual(s['data'][m.cell(0,0)],0)
        self.assertEqual(s['data'][m.cell(.2,0)],-1)

    def test_long_valid_ray_reveals_only_local_free_space_without_fake_obstacle(self):
        m=LiveMap(mapping_radius=.8)
        m.update(pose(),now=1);m.update(ranges(front=2000),now=1)
        s=m.snapshot(1)
        self.assertEqual(s['data'][m.cell(.7,0)],0)
        self.assertEqual(s['data'][m.cell(.8,0)],0)
        self.assertEqual(s['data'][m.cell(1,0)],-1)
        self.assertEqual(s['stats']['hits'],0)
        self.assertEqual(s['stats']['free_rays'],1)
        m.update(ranges(front=32766),now=2)
        self.assertEqual(m.snapshot(2)['data'][m.cell(0,.5)],-1)

if __name__=='__main__': unittest.main()
