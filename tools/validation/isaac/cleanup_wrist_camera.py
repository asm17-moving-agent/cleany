"""RTX wrist RGB using the existing nominal simulation mount and camera contract."""
import math
from pathlib import Path

import numpy as np
import yaml
from pxr import Gf, UsdGeom
from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


class WristCameras:
    """Publish selected rendered RGB; wrist depth is deliberately not supplied."""

    def __init__(self, stage, node, output):
        directory = Path('/main/ros2_ws/src/cleany_mujoco_sim/config')
        mounts = yaml.safe_load((directory/'sorting_wrist_cameras.yaml').read_text())
        contract = yaml.safe_load((directory/'wrist_camera.yaml').read_text())['camera_rendering']
        if mounts['source'] != 'simulation_nominal_cad':
            raise ValueError('Wrist cameras require explicit nominal simulation mounts')
        self.width, self.height = contract['width'], contract['height']
        self.focal = 12./math.tan(math.radians(contract['fovy_deg'])/2)
        self.output = output
        self.cameras, self.publishers, paths = [], {}, []
        self.counts = {'left':0, 'right':0}
        for arm in ('left', 'right'):
            mount = mounts[arm]
            parents = [p for p in stage.Traverse() if p.GetName() == mount['parent_frame']]
            if len(parents) != 1:
                raise RuntimeError(f'Expected one {arm} wrist parent')
            path = str(parents[0].GetPath())+'/NominalWristOptical'
            xf = UsdGeom.Xform.Define(stage, path)
            xf.AddTranslateOp().Set(Gf.Vec3d(*mount['translation_m']))
            x, y, z, w = mount['quaternion_xyzw']
            xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x,y,z)))
            path += '/IsaacCamera'
            self.cameras.append(RtxCamera(path, tick_rate=0.))
            camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
            transform = UsdGeom.Xformable(camera)
            transform.ClearXformOpOrder()
            transform.AddRotateXOp().Set(180.)
            camera.CreateFocalLengthAttr(self.focal)
            camera.CreateHorizontalApertureAttr(32.)
            camera.CreateVerticalApertureAttr(24.)
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 20.))
            camera.CreateFStopAttr(0.)
            paths.append(path)
            self.publishers[arm] = (
                node.create_publisher(Image, f'/{arm}_wrist_camera/image_raw', 2),
                node.create_publisher(CameraInfo, f'/{arm}_wrist_camera/camera_info', 2))
        self.sensor = TiledCameraSensor(paths, resolution=(self.height,self.width), annotators=['rgb'])

    def publish(self, seconds, active):
        if active == 'head':
            return
        if active not in self.publishers:
            raise ValueError(f'Unknown active camera: {active}')
        data, _ = self.sensor.get_data('rgb')
        if data is None:
            return
        data = np.asarray(data.numpy() if hasattr(data,'numpy') else data)
        rgb = data[('left','right').index(active)]
        if rgb.shape != (self.height,self.width,3):
            raise RuntimeError(f'Unexpected wrist image shape {rgb.shape}')
        header = Header(stamp=Time(sec=int(seconds),nanosec=round(seconds%1*1e9)),
                        frame_id=f'{active}_wrist_rgb_optical_frame')
        fx = fy = self.height*self.focal/24.
        cx, cy = self.width/2., self.height/2.
        image = Image(header=header,height=self.height,width=self.width,encoding='rgb8',
                      step=self.width*3,data=rgb.astype(np.uint8).tobytes())
        info = CameraInfo(header=header,height=self.height,width=self.width,
            distortion_model='plumb_bob',d=[0.]*5,
            k=[fx,0.,cx,0.,fy,cy,0.,0.,1.],r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.])
        image_pub, info_pub = self.publishers[active]
        image_pub.publish(image)
        info_pub.publish(info)
        self.counts[active] += 1
        if self.counts[active] in (1,5,50) or self.counts[active] % 100 == 0:
            from PIL import Image as PILImage
            directory = self.output/'media'
            directory.mkdir(exist_ok=True)
            PILImage.fromarray(rgb).save(directory/f'{active}-wrist-{self.counts[active]:06}.png')
