"""C3 joint waypoint traversal; only path endpoints are constrained to rest."""
import math
import numpy as np
from continuous_trajectory import clamped_c2_segments
from septic_trajectory import derivative,hermite_rows,septic,split_rows,continuity_metrics

def blended_movej(p,waypoints):
 q=np.asarray([p.positions(x) for x in waypoints],float)
 if len(q)<3:raise ValueError('at least three waypoints required')
 duration=[max(1.,max(2.5*abs(b-a)/np.asarray(p.velocity))) for a,b in zip(q,q[1:])]
 for _ in range(40):
  ticks=[max(2,math.ceil(t/p.tick)) for t in duration];h=np.array(ticks)*p.tick
  try:
   base=clamped_c2_segments(p,q.tolist(),ticks)
   v=np.array([base[0]['start_velocity_rad_s']]+[x['end_velocity_rad_s'] for x in base])
   acc=np.array([[derivative(base[0]['coefficients_rad'][a],0.,2,h[0]) for a in range(7)]]+
       [[derivative(x['coefficients_rad'][a],1.,2,t) for a in range(7)] for x,t in zip(base,h)])
   left=np.array([[derivative(x['coefficients_rad'][a],0.,3,t) for a in range(7)] for x,t in zip(base,h)])
   right=np.array([[derivative(x['coefficients_rad'][a],1.,3,t) for a in range(7)] for x,t in zip(base,h)])
   jerk=np.vstack((np.zeros(7),(right[:-1]+left[1:])/2,np.zeros(7)))
   v[0]=v[-1]=0;acc[0]=acc[-1]=0
   plans=[septic(p,hermite_rows(q[i],q[i+1],v[i],v[i+1],acc[i],acc[i+1],jerk[i],jerk[i+1],t),t) for i,t in enumerate(h)]
   plans=[part for plan in plans for part in split_rows(p,plan['coefficients_rad'],plan['ticks'])]
   return plans,dict(duration_s=sum(x['duration_s'] for x in plans),interior_speed_norm_rad_s=np.linalg.norm(v[1:-1,:6],axis=1).tolist(),
       reference_continuity=continuity_metrics(plans,p),quantized_continuity=continuity_metrics(plans,p,True),
       stops_at_every_waypoint=False)
  except ValueError as exc:last=exc;duration=[x*1.12 for x in duration]
 raise ValueError('blended path retiming exhausted: '+str(last))
