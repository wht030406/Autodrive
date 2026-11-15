"""Small deterministic checks for V2I fallback and signal safety."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from vanet.v2i_glosa.advice import read_display_status, read_speed_advice
from vanet.v2i_glosa.glosa import horizontal_speed_kmh, phase_at, red_safety_cap, speed_advice


class AdviceTests(unittest.TestCase):
    def test_stale_packet_returns_to_a(self):
        packet=dict(type='v2i.spat',sumo_time=0.,signals=[
            dict(stop_x=62.8,phase=2,remaining=10.)])
        self.assertIsNone(speed_advice(packet,2.01,18.,50.))
        self.assertLess(speed_advice(packet,1.,18.,50.),50.)

    def test_current_red_caps_both_vehicles(self):
        self.assertIsNone(red_safety_cap(('G','G','G'),40.))
        self.assertLess(red_safety_cap(('r','G','G'),40.),50.)

    def test_file_requires_matching_actor_and_expiry(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'advice.json'
            path.write_text(json.dumps(dict(actor_id=4,speed_kmh=0.,expires_wall=102.,
                                            packet_sequence=3)))
            self.assertEqual(read_speed_advice(path,4,now=101.),0.)
            self.assertIsNone(read_speed_advice(path,5,now=101.))
            self.assertIsNone(read_speed_advice(path,4,now=103.))
            self.assertEqual(read_display_status(path,4,now=101.)['packet_sequence'],3)

    def test_vertical_spawn_motion_is_not_driving_speed(self):
        self.assertEqual(horizontal_speed_kmh(SimpleNamespace(x=0.,y=0.,z=5.)),0.)
        self.assertAlmostEqual(horizontal_speed_kmh(SimpleNamespace(x=3.,y=4.,z=5.)),18.)

    def test_phase_boundary(self):
        self.assertEqual(phase_at(0,10.,11.),(1,2.))


if __name__=='__main__':
    unittest.main()
