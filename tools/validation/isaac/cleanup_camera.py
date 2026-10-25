"""Isaac RTX RGB-D adapter using the robot's actual optical frame.

Only rendered RGB and metric depth enter perception. No segmentation labels or
object transforms are published on sensor topics.
"""
import json
import numpy as np
from pxr import Gf, UsdGeom, UsdLux
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor
import omni.replicator.core as rep
from sensor_msgs.msg import CameraInfo, Image
from builtin_interfaces.msg import Time
from std_msgs.msg import Header


class CleanupCamera:
    def __init__(self, stage, node, output):
        self.node, self.output = node, output
        spec = json.loads((output/'config/room.json').read_text())['cleanup']
        self.width, self.height = spec.get('camera_resolution',[640,480])
        self.frame = 'head_camera_rgb_optical_frame'
        matches = [p for p in stage.Traverse() if p.GetName() == self.frame]
        if len(matches) != 1:
            raise RuntimeError('Expected exactly one robot head optical frame')
        path = str(matches[0].GetPath()) + '/IsaacCamera'
        self.camera = RtxCamera(path, tick_rate=0.)
        prim = UsdGeom.Camera(stage.GetPrimAtPath(path))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        # USD looks along -Z with +Y up; ROS optical looks +Z with +Y down.
        xf.AddRotateXOp().Set(180.)
        self.focal = spec.get('camera_focal_length_mm',24.)
        prim.CreateFocalLengthAttr(self.focal)
        prim.CreateHorizontalApertureAttr(32.)
        prim.CreateVerticalApertureAttr(24.)
        prim.CreateClippingRangeAttr(Gf.Vec2f(.05, 20.))
        prim.CreateFStopAttr(0.)
        prim.CreateShutterOpenAttr(0.)
        prim.CreateShutterCloseAttr(0.)
        self.sensor = TiledCameraSensor([path], resolution=(self.height, self.width),
                                        annotators=['rgb', 'distance_to_image_plane'])
        from cleanup_wrist_camera import WristCameras
        self.wrists = WristCameras(stage, node, output)
        UsdLux.DomeLight.Define(stage, '/CleanupLighting/Dome').CreateIntensityAttr(1800.)
        # The Facility ceiling blocks the sky. Add ceiling-mounted indoor lights
        # rather than hiding geometry from the robot's camera.
        for i, (x,y) in enumerate(((-.6,2.5),(-.6,0.),(-3.,2.5),(-3.,0.),
                                   (-6.,-3.),(-6.,-6.),(-9.,-3.),(-9.,-6.))):
            light = UsdLux.SphereLight.Define(stage, f'/CleanupLighting/Indoor_{i}')
            light.CreateIntensityAttr(1500.)
            light.CreateExposureAttr(5.)
            light.CreateRadiusAttr(.15)
            UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(x,y,2.4))
        rep.orchestrator.set_capture_on_play(False)
        self.rgb = node.create_publisher(Image, '/camera/color/image_raw', 2)
        self.depth = node.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 2)
        self.info = node.create_publisher(CameraInfo, '/camera/color/camera_info', 2)
        self.depth_info = node.create_publisher(CameraInfo, '/camera/aligned_depth_to_color/camera_info', 2)
        self.count = 0

    def capture(self, seconds, active_camera='head'):
        RenderingManager.render()
        rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False, wait_for_render=True)
        self.wrists.publish(seconds, active_camera)
        color, _ = self.sensor.get_data('rgb')
        depth, _ = self.sensor.get_data('distance_to_image_plane')
        if color is None or depth is None:
            return
        color = np.asarray(color.numpy() if hasattr(color, 'numpy') else color)[0]
        depth = np.asarray(depth.numpy() if hasattr(depth, 'numpy') else depth)[0].reshape(self.height, self.width)
        if color.shape != (self.height, self.width, 3):
            raise RuntimeError(f'Unexpected camera shape {color.shape}')
        stamp = Time(sec=int(seconds), nanosec=round((seconds%1)*1e9))
        header = Header(stamp=stamp, frame_id=self.frame)
        rgb = Image(header=header, height=self.height, width=self.width, encoding='rgb8',
                    is_bigendian=0, step=self.width*3, data=color.astype(np.uint8).tobytes())
        depth_image = Image(header=header, height=self.height, width=self.width, encoding='32FC1',
                    is_bigendian=0, step=self.width*4, data=depth.astype('<f4').tobytes())
        fx = fy = self.width*self.focal/32.
        cx, cy = self.width/2., self.height/2.
        info = CameraInfo(header=header, height=self.height, width=self.width,
            distortion_model='plumb_bob', d=[0.]*5,
            k=[fx,0.,cx,0.,fy,cy,0.,0.,1.], r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.])
        self.rgb.publish(rgb)
        self.depth.publish(depth_image)
        self.info.publish(info)
        self.depth_info.publish(info)
        self.count += 1
        if self.count in (5, 50) or self.count % 100 == 0:
            from PIL import Image as PILImage
            directory = self.output/'media'
            directory.mkdir(exist_ok=True)
            PILImage.fromarray(color).save(directory/f'head-{self.count:06}.png')
            np.save(directory/f'depth-{self.count:06}.npy', depth)
            (directory/f'head-{self.count:06}.json').write_text(json.dumps(dict(
                simulation_s=seconds, frame_id=self.frame, active_camera=active_camera,
                width=self.width, height=self.height, k=list(info.k))))
