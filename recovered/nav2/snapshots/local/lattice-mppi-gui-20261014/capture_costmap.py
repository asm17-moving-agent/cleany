from pathlib import Path
import json,time,math
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path as NavPath,Odometry
from tf2_ros import Buffer,TransformListener
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as MPath
from matplotlib.patches import Polygon
out=Path(__file__).parent/'costmap-stall';out.mkdir(exist_ok=True)
rclpy.init();n=Node('inspect_stalled_costmap');buf=Buffer();listener=TransformListener(buf,n);msgs={}
q=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE,reliability=ReliabilityPolicy.RELIABLE)
subs=[n.create_subscription(PolygonStamped,'/local_costmap/published_footprint',lambda m:msgs.update(footprint=m),q),n.create_subscription(NavPath,'/plan',lambda m:msgs.update(plan=m),q),n.create_subscription(Odometry,'/ground_truth/odom',lambda m:msgs.update(gt=m),10)]
start=time.monotonic()
while time.monotonic()-start<3:rclpy.spin_once(n,timeout_sec=.1)
client=n.create_client(GetCostmap,'/local_costmap/get_costmap');assert client.wait_for_service(timeout_sec=5)
f=client.call_async(GetCostmap.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=8);m=f.result().map
meta=m.metadata;res=meta.resolution;ox=meta.origin.position.x;oy=meta.origin.position.y;a=np.array(m.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x)
t=buf.lookup_transform(m.header.frame_id,'base_link',rclpy.time.Time());rot=t.transform.rotation;yaw=math.atan2(2*(rot.w*rot.z+rot.x*rot.y),1-2*(rot.y**2+rot.z**2));pose=[t.transform.translation.x,t.transform.translation.y,yaw]
fp=msgs.get('footprint');assert fp and fp.header.frame_id==m.header.frame_id
poly=np.array([[p.x,p.y] for p in fp.polygon.points]);yy,xx=np.indices(a.shape);centers=np.c_[ox+(xx.ravel()+.5)*res,oy+(yy.ravel()+.5)*res];inside=MPath(poly).contains_points(centers).reshape(a.shape)
lethal=np.argwhere((a==254)&inside);unknown=np.argwhere((a==255)&inside)
payload={'frame':m.header.frame_id,'stamp':{'sec':m.header.stamp.sec,'nanosec':m.header.stamp.nanosec},'resolution':res,'origin':[ox,oy],'shape':list(a.shape),'robot_pose':pose,'published_footprint':poly.tolist(),'footprint_stamp':{'sec':fp.header.stamp.sec,'nanosec':fp.header.stamp.nanosec},'lethal_cell_centers_inside_footprint':[[ox+(x+.5)*res,oy+(y+.5)*res] for y,x in lethal],'unknown_cell_centers_inside_footprint':len(unknown),'cost_counts_inside_footprint':{str(int(k)):int(v) for k,v in zip(*np.unique(a[inside],return_counts=True))},'note':'Live asynchronous costmap/TF/footprint snapshot; cell-center overlap is diagnostic, not an exact reproduction of MPPI boundary collision checking.'}
np.save(out/'local-costmap.npy',a)
# Transform current plan from map into the costmap frame.
plan=msgs.get('plan');path=[]
if plan:
 tr=buf.lookup_transform(m.header.frame_id,plan.header.frame_id,rclpy.time.Time());rr=tr.transform.rotation;theta=math.atan2(2*(rr.w*rr.z+rr.x*rr.y),1-2*(rr.y**2+rr.z**2));c,s=math.cos(theta),math.sin(theta)
 path=[[c*p.pose.position.x-s*p.pose.position.y+tr.transform.translation.x,s*p.pose.position.x+c*p.pose.position.y+tr.transform.translation.y] for p in plan.poses]
payload['plan_in_costmap_frame']=path
(out/'snapshot.json').write_text(json.dumps(payload,indent=2))
fig,axes=plt.subplots(1,2,figsize=(13,6));extent=[ox,ox+a.shape[1]*res,oy,oy+a.shape[0]*res]
for ax in axes:
 ax.imshow(np.where(a==255,np.nan,a),origin='lower',extent=extent,cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest')
 ly,lx=np.where(a==254);ax.scatter(ox+(lx+.5)*res,oy+(ly+.5)*res,c='black',s=10,marker='s',label='Lethal cost 254')
 ax.add_patch(Polygon(poly,fill=False,edgecolor='#00a8e8',linewidth=2.5,label='Published padded footprint'))
 if path:
  p=np.array(path);ax.plot(p[:,0],p[:,1],color='#247a32',lw=2,label='Smac 2D path')
 if len(lethal):
  pts=np.array(payload['lethal_cell_centers_inside_footprint']);ax.scatter(pts[:,0],pts[:,1],s=110,facecolors='none',edgecolors='#ff00df',lw=2,label='Lethal centers inside footprint')
 ax.arrow(pose[0],pose[1],.3*math.cos(yaw),.3*math.sin(yaw),head_width=.09,color='#0088cc');ax.set_aspect('equal');ax.set_xlabel(m.header.frame_id+' X (m)');ax.set_ylabel(m.header.frame_id+' Y (m)')
axes[0].set_xlim(ox,extent[1]);axes[0].set_ylim(oy,extent[3]);axes[0].set_title('Live local costmap / stopped robot');axes[0].legend(fontsize=8,loc='upper left')
axes[1].set_xlim(pose[0]-1,pose[0]+1);axes[1].set_ylim(pose[1]-1,pose[1]+1);axes[1].set_title(f'Footprint detail: {len(lethal)} lethal cell centers inside')
fig.suptitle('Smac 2D + MPPI: stalled pose costmap inspection',fontsize=15);fig.tight_layout();fig.savefig(out/'costmap-stall.png',dpi=160)
print(json.dumps({k:v for k,v in payload.items() if k not in ['plan_in_costmap_frame']},indent=2));n.destroy_node();rclpy.shutdown()
