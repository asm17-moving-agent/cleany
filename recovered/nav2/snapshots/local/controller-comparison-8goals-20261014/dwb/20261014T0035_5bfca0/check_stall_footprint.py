import ast,json,math,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
p=Path(sys.argv[1]);s=json.loads((p/'snapshot.json').read_text());m=deserialize_message((p/'_local_costmap_costmap.cdr').read_bytes(),OccupancyGrid);pose=s['topics']['/odom']['pose'];x,y,t=pose;fp=[[-.338525,-.427373],[-.338525,.427427],[.338475,.427427],[.338475,-.427373]];c,si=math.cos(t),math.sin(t);poly=[(x+c*a-si*b,y+si*a+c*b) for a,b in fp];axes=[(1,0),(0,1)]+[(poly[(i+1)%4][1]-a[1],a[0]-poly[(i+1)%4][0]) for i,a in enumerate(poly)];cells=[]
for row in range(m.info.height):
 for col in range(m.info.width):
  value=m.data[row*m.info.width+col]
  if value!=100 and value>=0:continue
  cx=m.info.origin.position.x+(col+.5)*m.info.resolution;cy=m.info.origin.position.y+(row+.5)*m.info.resolution;half=m.info.resolution/2;free=False
  for nx,ny in axes:
   ps=[a*nx+b*ny for a,b in poly];center=cx*nx+cy*ny;extent=half*(abs(nx)+abs(ny))
   if center+extent<min(ps) or center-extent>max(ps):free=True;break
  if not free:cells.append({'cell':[col,row],'center_odom':[cx,cy],'occupancy':value})
r={'snapshot_is_asynchronous':True,'local_costmap_stamp_sim_s':s['topics']['/local_costmap/costmap']['stamp_sim_s'],'odom_stamp_sim_s':s['topics']['/odom']['stamp_sim_s'],'padded_full_footprint':fp,'lethal_or_unknown_cells_intersecting_padded_footprint':cells,'interpretation':'Read-only sampled local footprint overlap; supports a collision-admissibility explanation but is not an atomic recording of a DWB scoring cycle and does not isolate which layer or critic caused every rejection.'};(p/'footprint_check.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
