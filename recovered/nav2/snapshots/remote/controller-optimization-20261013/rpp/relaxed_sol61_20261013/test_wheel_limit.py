import math,random,unittest
from wheel_limit import limit_twist,wheel_rates,slew_twist
class WheelLimitTest(unittest.TestCase):
    def test_coupled_bound_and_direction(self):
        random.seed(228)
        for _ in range(1000):
            request=[random.uniform(-1,1) for _ in range(3)]
            output,scale=limit_twist(request,5.)
            self.assertLessEqual(max(map(abs,wheel_rates(output))),5.+1e-12)
            # The cross-product checks collinearity of the entire twist.
            self.assertAlmostEqual(output[0]*request[1]-output[1]*request[0],0.)
            self.assertAlmostEqual(output[0]*request[2]-output[2]*request[0],0.)
            if scale<1:self.assertAlmostEqual(max(map(abs,wheel_rates(output))),5.)
    def test_physical_axis_extrema(self):
        x,_=limit_twist([1,0,0],5.)
        y,_=limit_twist([0,1,0],5.)
        z,_=limit_twist([0,0,1],5.)
        self.assertAlmostEqual(x[0],.3175);self.assertAlmostEqual(y[1],.3175)
        self.assertAlmostEqual(z[2],.3175/((.35+.6038)/2))
        self.assertEqual(limit_twist([0,0,0],5.),([0,0,0],1.))
    def test_slew_step_bound_and_wheel_convexity(self):
        previous=[0.,0.,0.]
        for request in [[.30,0,-.33],[.30,0,0],[0,0,.35],[0,0,-.35],[0,0,0]]:
            target,_=limit_twist(request,4.8)
            for _ in range(50):
                output,_=slew_twist(previous,target,.05,8.)
                delta=[b-a for a,b in zip(previous,output)]
                self.assertLessEqual(max(map(abs,wheel_rates(delta))),.4+1e-12)
                self.assertLessEqual(max(map(abs,wheel_rates(output))),4.8+1e-12)
                previous=output
            for a,b in zip(output,target):self.assertAlmostEqual(a,b)
    def test_invalid(self):
        for cap in [0,-1,5.01,math.nan,math.inf]:
            with self.assertRaises(ValueError):limit_twist([0,0,0],cap)
        with self.assertRaises(ValueError):limit_twist([math.nan,0,0],5.)
if __name__=='__main__':unittest.main()
