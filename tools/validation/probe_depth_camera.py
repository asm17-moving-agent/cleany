#!/usr/bin/env python3
"""Capture depth evidence and timing without providing robot-frame obstacle claims."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace',default='/camera/d435')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('depth_evidence_probe')
    samples=[];images=[];info=[]
    def on_image(message: Image) -> None:
        dtype=np.dtype(('>'if message.is_bigendian else'<')+('u2'if message.encoding=='16UC1'else'f4'))
        if message.encoding not in ('16UC1','32FC1'):raise ValueError(message.encoding)
        rows=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
        depth=np.frombuffer(rows[:,:message.width*dtype.itemsize].copy().tobytes(),dtype=dtype).reshape(message.height,message.width).astype(float)
        if message.encoding=='16UC1':depth*=.001
        mask=np.isfinite(depth)&(depth>0)
        samples.append({'received':time.monotonic(),'stamp':message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                        'valid_fraction':float(mask.mean()),'frame':message.header.frame_id,
                        'width':message.width,'height':message.height,'encoding':message.encoding})
        images[:]=[depth]
    def on_info(message: CameraInfo) -> None:
        info[:]=[{'width':message.width,'height':message.height,'k':list(message.k),
                  'distortion_model':message.distortion_model,'d':list(message.d),'frame':message.header.frame_id}]
    node.create_subscription(Image,args.namespace+'/depth/image_rect_raw',on_image,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,args.namespace+'/depth/camera_info',on_info,qos_profile_sensor_data)
    end=time.monotonic()+12
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.2)
            if len(samples)>=60 and info:break
    finally:node.destroy_node();rclpy.shutdown()
    if len(samples)<2 or not info:raise RuntimeError('Depth or CameraInfo missing')
    depth=images[0];valid=np.isfinite(depth)&(depth>0)
    np.save(args.output/'depth_m.npy',depth)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(10,7),layout='constrained')
    im=ax.imshow(np.ma.masked_where(~valid,depth),vmin=.2,vmax=3,cmap='turbo')
    ax.set_title('D435 optical depth (m) — not height above floor')
    fig.colorbar(im,ax=ax,label='Depth (m)');fig.savefig(args.output/'depth-preview.png',dpi=140);plt.close(fig)
    report={'samples':len(samples),'received_hz':(len(samples)-1)/(samples[-1]['received']-samples[0]['received']),
            'stamp_hz':(len(samples)-1)/(samples[-1]['stamp']-samples[0]['stamp']),
            'valid_fraction_median':float(np.median([x['valid_fraction']for x in samples])),
            'depth_percentiles_m':np.percentile(depth[valid],[5,50,95]).tolist()if valid.any()else[],
            'camera_info':info[0],'frames':samples,
            'boundary':'Optical depth and fresh-frame reception only; no base transform or obstacle detection validation'}
    (args.output/'depth-probe.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items()if k!='frames'}))


if __name__=='__main__':main()
