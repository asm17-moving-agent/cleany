import math
import random
import unittest
from wheel_limiter import limit, wheel_rates, slew

class EnvelopeTests(unittest.TestCase):
    def test_physical_axis_limits(self):
        for command in ((.3175,0,0),(0,.3175,0),(0,0,.3175/((.35+.6038)/2))):
            out,scale=limit(command)
            self.assertAlmostEqual(scale,1)
            self.assertAlmostEqual(max(map(abs,wheel_rates(out))),5)
    def test_combined_cap_preserves_every_component_ratio(self):
        rng=random.Random(227)
        for _ in range(20000):
            command=tuple(rng.uniform(-3,3) for _ in range(3))
            out,scale=limit(command)
            self.assertLessEqual(max(map(abs,wheel_rates(out))),5+1e-12)
            for x,y in zip(command,out):self.assertAlmostEqual(y,x*scale)
    def test_transitions_preserve_cap_and_bound_each_wheel_acceleration(self):
        rng=random.Random(227)
        previous=(0.,0.,0.)
        for _ in range(20000):
            target,_=limit(tuple(rng.uniform(-3,3) for _ in range(3)),4.99)
            dt=rng.uniform(.001,.05)
            out,_=slew(previous,target,dt,8.)
            self.assertLessEqual(max(map(abs,wheel_rates(out))),4.99+1e-12)
            changes=wheel_rates(tuple(b-a for a,b in zip(previous,out)))
            self.assertLessEqual(max(map(abs,changes)),8*dt+1e-12)
            previous=out
    def test_zero_and_invalid_stop(self):
        self.assertEqual(limit((0,0,0)),((0,0,0),1))
        for v in (math.nan,math.inf):self.assertEqual(limit((v,0,0))[0],(0,0,0))

if __name__=='__main__':unittest.main()
