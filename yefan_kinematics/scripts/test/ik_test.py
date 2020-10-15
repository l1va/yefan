import unittest
import numpy.testing
from yefan_kinematics.scripts.ik import calculate as ik_calc
from yefan_kinematics.scripts.fk import calculate as fk_calc
from yefan_kinematics.scripts.test.fk_test import IFK_CASES

class TestIK(unittest.TestCase):
    def test_ik(self):

        for i in range(len(IFK_CASES)):
            c = IFK_CASES[i]
            res_ik = ik_calc(c[1])
            res_fk = fk_calc(res_ik)
            numpy.testing.assert_almost_equal(c[1], res_fk, decimal=4, err_msg="case failed: " + str(i))
            #
            # def test_positions_failed(self):
            #     with self.assertRaises(KeyError) as ctx:
            #         init_js_wrapper().position('joint3')
            #     self.assertIn("not found in names", str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
