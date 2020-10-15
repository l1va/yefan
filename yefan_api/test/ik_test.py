import unittest
import numpy.testing
from yefan_api.ik import calculate
from yefan_api.test.fk_test import IFK_CASES

class TestIK(unittest.TestCase):
    def test_ik(self):

        for i in range(len(IFK_CASES)):
            c = IFK_CASES[i]
            numpy.testing.assert_almost_equal(c[0], calculate(c[1]), decimal=4, err_msg="case failed: " + str(i))
            #
            # def test_positions_failed(self):
            #     with self.assertRaises(KeyError) as ctx:
            #         init_js_wrapper().position('joint3')
            #     self.assertIn("not found in names", str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
