"""M6 shadow recovery on recorded frames, and refusal of insufficient visual evidence."""
import base64
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from workcell.wrist_servo_host import Servo, locate_m6_screw, m6_screw_consensus


FIXTURES = Path(__file__).parent / 'fixtures' / 'm6_shadow'
EXPECTED = [271.89919957876685, 207.34043578675406]


def recorded(prefix):
    return [cv2.imread(str(p), cv2.IMREAD_GRAYSCALE) for p in sorted(FIXTURES.glob(prefix + '_frame_*.png'))]


def scene(head=True, shadow=True, angle=0):
    gray = np.full((400, 640), 165, np.uint8)
    if shadow:
        cv2.rectangle(gray, (304, 165), (345, 300), 90, -1)
    cv2.rectangle(gray, (290, 175), (308, 285), 235, -1)
    if head:
        cv2.rectangle(gray, (282, 165), (316, 190), 235, -1)
    gray = cv2.GaussianBlur(gray, (0, 0), 1.5)
    return cv2.warpAffine(gray, cv2.getRotationMatrix2D((300, 230), angle, 1.), (640, 400), borderValue=165)


class M6Shadow(unittest.TestCase):
    def test_recorded_failures_have_consistent_heads_and_grip_points(self):
        references = json.loads((FIXTURES / 'provenance.json').read_text())
        for ref in references:
            with self.subTest(frame=ref['file']):
                gray = cv2.imread(str(FIXTURES / ref['file']), 0)
                obs = locate_m6_screw(gray, EXPECTED, 220.)
                self.assertTrue(obs['shadow_removed'])
                self.assertFalse(obs['head_from_hint'])
                self.assertFalse(obs['cut'])
                self.assertLess(obs['head_dir'][1], -.95)
                self.assertLess(np.linalg.norm(np.subtract(obs['center_px'], ref['grip_reference_px'])), 12.)
        for prefix in ('retry1', 'retry2'):
            group = m6_screw_consensus(recorded(prefix), EXPECTED, 220.)
            self.assertEqual(len(group), 3)
            self.assertLess(np.max(np.ptp([o['center_px'] for o in group], axis=0)), 1.)

    def test_head_direction_rotates_with_the_body(self):
        for angle in (0, 45, 90, 180):
            with self.subTest(angle=angle):
                obs = locate_m6_screw(scene(angle=angle), [300, 230], 100.)
                expected = np.array([-np.sin(np.deg2rad(angle)), -np.cos(np.deg2rad(angle))])
                self.assertGreater(float(np.dot(obs['head_dir'], expected)), .95)

    def test_equal_ends_are_still_rejected(self):
        with self.assertRaisesRegex(ValueError, 'head direction ambiguous'):
            locate_m6_screw(scene(head=False, shadow=False), [300, 230], 100.)

    def test_empty_tray_glints_and_nut_are_not_screws(self):
        for kind in ('empty', 'glints', 'nut'):
            with self.subTest(kind=kind):
                gray = np.full((400, 640), 165, np.uint8)
                if kind == 'glints':
                    for x in range(240, 360, 25):
                        for y in range(160, 300, 25):
                            cv2.circle(gray, (x, y), 6, 245, -1)
                if kind == 'nut':
                    cv2.circle(gray, (300, 230), 28, 235, -1)
                    cv2.circle(gray, (300, 230), 12, 70, -1)
                with self.assertRaises(ValueError):
                    locate_m6_screw(cv2.GaussianBlur(gray, (0, 0), 1.5), [300, 230], 100.)

    def test_small_highlight_on_an_ambiguous_body_is_not_enough(self):
        gray = np.full((400, 640), 165, np.uint8)
        cv2.rectangle(gray, (280, 160), (320, 290), 85, -1)
        cv2.rectangle(gray, (285, 170), (300, 190), 240, -1)
        with self.assertRaises(ValueError):
            locate_m6_screw(cv2.GaussianBlur(gray, (0, 0), 1.5), [300, 230], 100.)

    def test_two_nearby_screws_are_still_ambiguous(self):
        gray = np.full((400, 640), 165, np.uint8)
        body = scene(shadow=False)[:, 270:355]
        gray[:, 220:305] = body
        gray[:, 320:405] = body
        with self.assertRaisesRegex(ValueError, 'More than one'):
            locate_m6_screw(gray, [300, 230], 120.)

    def test_one_valid_frame_cannot_authorize_a_grasp(self):
        blank = np.full((400, 640), 165, np.uint8)
        with self.assertRaisesRegex(ValueError, 'detected 1/3'):
            m6_screw_consensus([recorded('retry1')[0], blank, blank], EXPECTED, 220.)

    def test_noisy_third_frame_does_not_remove_two_matching_observations(self):
        frames = recorded('retry1')[:2] + [np.full((400, 640), 165, np.uint8)]
        self.assertEqual(len(m6_screw_consensus(frames, EXPECTED, 220.)), 2)

    def test_conflicting_grip_points_are_still_rejected(self):
        with patch('workcell.wrist_servo_host.locate_m6_screw', side_effect=[
                dict(center_px=[100, 200]), dict(center_px=[150, 200]), dict(center_px=[200, 200])]):
            with self.assertRaisesRegex(ValueError, 'grip points disagree'):
                m6_screw_consensus([None] * 3, EXPECTED, 220.)

    def test_real_servo_path_accepts_images_but_retains_iteration_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'servo_config.json').write_text(json.dumps(dict(
                job_sha256='offline', part_class='screw', part_size='m6',
                J_pixels_per_m=[[4973., -306.], [-422., -4540.]],
                coarse_center_px=EXPECTED, target_px=EXPECTED)))
            servo = Servo(root)
            servo.next_iteration = 4
            images = [cv2.imencode('.png', gray)[1].tobytes() for gray in recorded('retry2')]
            reply = servo.respond(dict(nonce='offline', job_sha256='offline', iteration=4,
                camera_session='offline', cumulative_tool_xy_m=[0., 0.],
                images=[base64.b64encode(raw).decode() for raw in images],
                frame_sha256=[hashlib.sha256(raw).hexdigest() for raw in images]))
            self.assertEqual(reply['action'], 'RETURN')
            self.assertEqual(reply['reason'], 'Iteration limit')
            self.assertGreater(reply['pixel_error'], 5.)
            self.assertEqual(len(reply['observations']), 3)


if __name__ == '__main__':
    unittest.main()
