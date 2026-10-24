"""Analyze full recorded physics contacts, independently from live observer QoS."""
import json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from ros_gz_interfaces.msg import Contacts

ROOT=Path(__file__).resolve().parent

def analyze(run):
    result=json.loads((run/'result.json').read_text())
    if not result.get('completed'):return
    start,end=result['navigation_sim_start_s'],result['navigation_sim_end_s']
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    topics=[x.name for x in reader.get_all_topics_and_types() if x.type=='ros_gz_interfaces/msg/Contacts']
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    counts={};floor=unknown=self_contacts=0;events=0;nonfloor=[];last={}
    while reader.has_next():
        topic,data,_=reader.read_next();msg=deserialize_message(data,Contacts)
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not start <= t <= end:continue
        counts[topic]=counts.get(topic,0)+1
        for p in msg.contacts:
            a,b=p.collision1.name,p.collision2.name
            if not a or not b:unknown+=1;continue
            if 'ground_plane' in a or 'ground_plane' in b:floor+=1;continue
            if a.startswith('cleany_mecanum::') and b.startswith('cleany_mecanum::'):self_contacts+=1;continue
            key=tuple(sorted((a,b)))
            if key not in last or t-last[key]>.25:events+=1
            last[key]=t
            nonfloor.append({'sim_s':t,'a':a,'b':b,'max_depth_m':max(p.depths,default=None)})
    available=floor>0 and unknown==0
    summary={'source':'full rosbag2 Contacts topics; navigation window by source sim stamp','counts_by_topic':counts,
             'ground_pair_samples':floor,'self_pair_samples':self_contacts,'unnamed_pair_samples':unknown,
             'nonfloor_pair_samples':len(nonfloor),'nonfloor_pair_episodes':events if available else None,
             'max_nonfloor_depth_m':max((p['max_depth_m'] for p in nonfloor if p['max_depth_m'] is not None),default=None),
             'actual_contact_count':events if available else None,
             'null_reason':None if available else 'No floor positive control or unnamed physics collision entities'}
    (run/'raw_contact_summary.json').write_text(json.dumps(summary,indent=2))
    (run/'raw_nonfloor_contacts.json').write_text(json.dumps(nonfloor))
    print(json.dumps({'run':str(run),'contacts':summary['actual_contact_count'],'ground_samples':floor,'nonfloor_samples':len(nonfloor),'null_reason':summary['null_reason']}),flush=True)

if __name__=='__main__':
    for p in ROOT.glob('*/*/result.json'):
        if (p.parent/'bag/metadata.yaml').exists() and not (p.parent/'raw_contact_summary.json').exists():analyze(p.parent)
