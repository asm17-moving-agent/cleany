from pathlib import Path
import sqlite3,json,sys,math
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
p=Path(sys.argv[1]);scenario=json.loads((p/'scenario.json').read_text());spawn=scenario['spawn_pose']
for f in (p/'bag').glob('*.db3'):
 c=sqlite3.connect('file:'+str(f)+'?mode=ro',uri=True)
 for tid,name in c.execute('select id,name from topics').fetchall():
  if name not in ['/ground_truth/odom','/head_pan/state','/body_guard_3d/state']:continue
  rows=c.execute('select data from messages where topic_id=? order by id desc limit 1',(tid,)).fetchall()
  if not rows:continue
  m=deserialize_message(rows[0][0],Odometry if name=='/ground_truth/odom' else String)
  print(name,[math.cos(spawn[5])*(m.pose.pose.position.x-spawn[0])+math.sin(spawn[5])*(m.pose.pose.position.y-spawn[1]),-math.sin(spawn[5])*(m.pose.pose.position.x-spawn[0])+math.cos(spawn[5])*(m.pose.pose.position.y-spawn[1]),m.header.stamp.sec+m.header.stamp.nanosec*1e-9] if name=='/ground_truth/odom' else m.data)
