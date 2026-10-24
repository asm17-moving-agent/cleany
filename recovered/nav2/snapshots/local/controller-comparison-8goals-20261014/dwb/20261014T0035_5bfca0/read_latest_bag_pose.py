import json,math,sqlite3,sys,time
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
p=Path(sys.argv[1]);assert (p/'result.json').exists(), 'Do not read SQLite while the recorder is running: reader locks can abort its writer';db=next((p/'bag').glob('*.db3'));c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);out={'captured_epoch_s':time.time(),'source':'read-only last committed bag record'}
for topic in ['/ground_truth/odom','/odom']:
 tid=c.execute('select id from topics where name=?',(topic,)).fetchone()[0];row=c.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit1'.replace('limit1','limit 1'),(tid,)).fetchone();m=deserialize_message(row[1],Odometry);q=m.pose.pose.orientation;v=m.twist.twist;out[topic]={'header_sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'bag_record_timestamp_ns':row[0],'pose':[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],'twist':[v.linear.x,v.linear.y,v.angular.z]}
c.close();(p/'latest_bag_pose.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
