import math,unittest
import numpy as np
from septic_trajectory import bernstein_integer_bounds,bernstein_bounds,extrema,derivative

class BernsteinTests(unittest.TestCase):
 def test_scalar_horner_matches_original_numpy_operation_order(self):
  from numpy.polynomial import polynomial as poly
  rng=np.random.default_rng(18)
  for _ in range(100):
   row=rng.uniform(-10,10,8)
   for u in [0.,1.,.37]:
    for k in range(4):
     old=float(poly.polyval(u,poly.polyder(row,k)))/.25**k
     self.assertEqual(derivative(row,u,k,.25),old)
 def test_integer_hull_contains_independent_polynomial_extrema(self):
  rng=np.random.default_rng(1810)
  for n in range(2,9):
   for _ in range(30):
    q=[int(x) for x in rng.integers(-1000000,1000000,n)]
    for order in range(min(3,n)):
     lo,hi,den=bernstein_integer_bounds(q,order)
     mn,mx=extrema(q,order)
     self.assertLessEqual(lo/den,mn+1e-6)
     self.assertGreaterEqual(hi/den,mx-1e-6)
 def test_exact_integer_convex_hull_and_constant_endpoint(self):
  # p(u)=u has Bernstein coefficients 0..1; no endpoint overrun.
  self.assertEqual(bernstein_integer_bounds([0,1,0,0,0,0,0,0]),(0,105,105))
  lo,hi,den=bernstein_integer_bounds([123,0,0,0,0,0,0,0])
  self.assertEqual(lo,123*den);self.assertEqual(hi,123*den)
 def test_float_metadata_encloses_exact_extrema(self):
  row=[.1,.03,-.2,.3,-.4,.2,-.05,.01]
  for order in range(4):
   lo,hi=bernstein_bounds(row,order,.25);mn,mx=extrema(row,order,.25)
   self.assertLessEqual(lo,mn);self.assertGreaterEqual(hi,mx)

if __name__=='__main__':unittest.main()
