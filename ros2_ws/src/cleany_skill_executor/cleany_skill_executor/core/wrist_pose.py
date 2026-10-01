"""Rigid held-object orientation prediction; not a visual measurement."""
import math
from geometry_msgs.msg import Quaternion


def carried_orientation(contact: Quaternion, current: Quaternion, original: Quaternion) -> Quaternion:
    def unit(q):
        values=(q.x,q.y,q.z,q.w)
        norm=math.sqrt(sum(v*v for v in values))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError('Invalid carried orientation')
        return tuple(v/norm for v in values)
    def mul(a,b):
        x,y,z,w=a; X,Y,Z,W=b
        return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)
    x,y,z,w=unit(contact)
    values=mul(mul(unit(current),(-x,-y,-z,w)),unit(original))
    return Quaternion(x=values[0],y=values[1],z=values[2],w=values[3])
