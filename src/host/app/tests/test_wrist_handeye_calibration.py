import unittest
import numpy as np
from workcell.wrist_handeye_calibration import fit

class EyeInHand(unittest.TestCase):
    def test_recovers_camera_mount_and_fixed_board_with_unseen_poses(self):
        rng=np.random.default_rng(25)
        def rigid():
            r,_=np.linalg.qr(rng.normal(size=(3,3)))
            if np.linalg.det(r)<0:r[:,0]*=-1
            t=np.eye(4);t[:3,:3]=r;t[:3,3]=rng.uniform(-.3,.3,3)
            return t
        x,y=rigid(),rigid();rows=[]
        for i in range(15):
            g=rigid();c=np.linalg.inv(x)@np.linalg.inv(g)@y
            rows.append(dict(name=str(i),T_base_end=g.tolist(),T_camera_board=c.tolist()))
        result=fit(rows[:10],rows[10:])
        np.testing.assert_allclose(result['T_end_camera'],x,atol=1e-10)
        np.testing.assert_allclose(result['T_base_board'],y,atol=1e-10)
        self.assertLess(result['holdout']['position_max_mm'],1e-8)
        self.assertFalse(result['robot_execution_authorized'])
        with self.assertRaisesRegex(ValueError,'OVERLAP'):fit(rows[:10],rows[9:])
        bad=[dict(s,T_camera_board=np.linalg.inv(s['T_camera_board']).tolist()) for s in rows]
        wrong=fit(bad[:10],bad[10:])
        self.assertGreater(wrong['holdout']['position_rmse_mm'],10.)

if __name__=='__main__':unittest.main()
