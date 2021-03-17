#! /usr/bin/env python3

import unittest
import rostest
from ecs.cpd import DiffRatio, VarRatio

class CPDTestCase(unittest.TestCase):

    def testDiffRatio(self):
        method = DiffRatio()
        for i in range(40):
            method.addValue(1)
        for i in range(40):
            method.addValue(3)
        self.assertEqual(method.isChangePoint(), True)

    def testVarRatio(self):
        method = VarRatio(var_stable=1.0)
        for i in range(20):
            method.addValue(2)
        for i in range(20):
            method.addValue(-2)        
        self.assertEqual(method.isChangePoint(), True)            

if __name__ == "__main__":
    rostest.unitrun("ecs", "cpd_tests", CPDTestCase)