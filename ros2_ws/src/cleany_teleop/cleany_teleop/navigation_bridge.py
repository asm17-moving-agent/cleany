"""Read map/localization and own one guarded Nav2 goal for the phone remote."""
import math
import json
import time
import uuid

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.task import Future
from cleany_interfaces.action import StopMission
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from cleany_mission_manager.adapters.supervision import RosSupervision
from cleany_mission_manager.core.supervision import Authorization
from .map_view import MapView
from .driver_transport import limit_wheel_velocity


class NavigationBridge:
    """Goal execution uses the existing Control permission/MCU ACK protocol.

    Automatic intent passes through motion/cmd_vel and the independent LiDAR
    monitor before the shared final publisher. Manual intent retains the
    existing operator-controlled path. This bridge never publishes WheelCommand.
    """
    def __init__(self,node,control,lock):
        self.node,self.control,self.lock=node,control,lock
        self.scan_timeout=float(node.declare_parameter('navigation.scan_timeout_s',.5).value)
        self.odom_timeout=float(node.declare_parameter('navigation.odom_timeout_s',.3).value)
        self.pose_timeout=float(node.declare_parameter('navigation.pose_timeout_s',.5).value)
        self.goal_distance=float(node.declare_parameter('navigation.goal_distance_m',.03).value)
        self.goal_angle=float(node.declare_parameter('navigation.goal_angle_rad',.1).value)
        self.max_wheel_speed=float(node.declare_parameter('navigation.max_wheel_rad_s',2.).value)
        if not all(math.isfinite(v) and v>0 for v in (self.scan_timeout,self.odom_timeout,self.pose_timeout,self.goal_distance,self.goal_angle)):
            raise ValueError('Navigation limits must be finite and positive')
        if not math.isfinite(self.max_wheel_speed) or not 0 < self.max_wheel_speed <= 10:
            raise ValueError('Navigation wheel speed must be in (0, 10] rad/s')
        self.map=None;self.path=[];self.path_at=0.;self.scan_at=0.;self.velocity_at=0.
        self.velocity=(0.,0.,0.);self.pose=None;self.pose_at=0.;self.odom_at=0.
        self.phase='idle';self.reason='Nav2 준비 대기';self.goal_id='';self.handle=None
        self.last_input_failure={}
        self.slip={};self.traction={};self.traction_at=0.
        self.localization={}
        self.pending=None;self.heartbeat=0.;self.sequence=0;self.last_update=0.
        self.epoch=0;self.outcome='cancelled';self.requests={};self.cancel_future=None;self.started_control=False
        self.supervision=RosSupervision(node,['NAVIGATION'],heartbeat_timeout=1.,request_timeout=1.)
        self.client=ActionClient(node,NavigateToPose,'navigate_to_pose')
        self.preview_client=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
        self.preview_generation=0;self.preview_id='';self.preview_phase='idle'
        self.preview_path=[];self.preview_error='';self.preview_handle=None;self.preview_target=None
        self.stop_waiters=[]
        self.stop_server=ActionServer(node,StopMission,'navigation/stop',
            execute_callback=self.execute_stop, goal_callback=self.admit_stop)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,node)
        self.nomotion=node.create_client(Empty,'request_nomotion_update')
        self.nomotion_future=None;self.nomotion_at=0.
        self.initial=node.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        node.create_subscription(OccupancyGrid,'map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Path,'plan',self.on_path,1)
        node.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        node.create_subscription(Odometry,'odom',self.on_odom,qos_profile_sensor_data)
        node.create_subscription(Twist,'nav2/cmd_vel',self.on_velocity,1)
        node.create_subscription(String,'navigation/slip',self.on_slip,1)
        node.create_subscription(String,'navigation/traction_recovery',self.on_traction,1)
        node.create_subscription(String,'localization/status',self.on_localization,1)

    def on_localization(self, message):
        try:
            value=json.loads(message.data)
            if isinstance(value,dict):
                with self.lock:self.localization=value
        except (ValueError,TypeError):pass

    def on_slip(self, message):
        try:
            value=json.loads(message.data)
            if not isinstance(value,dict):return
            with self.lock:self.slip=value
        except (ValueError,TypeError):pass

    def on_traction(self, message):
        try:
            value=json.loads(message.data)
            if not isinstance(value,dict):return
            with self.lock:self.traction=value;self.traction_at=time.monotonic()
        except (ValueError,TypeError):pass

    def admit_stop(self,request):
        with self.lock:
            current=(request.mission_id==self.goal_id and request.lease_id==self.supervision.lease_id)
            # After a process restart the Supervisor still owns the previous lease.
            # A new idle bridge can stop its output and verify MCU standstill without
            # inventing a goal or restoring the old motion authority.
            restarted_idle=(not self.goal_id and self.phase=='idle' and self.handle is None
                            and self.pending is None and bool(request.mission_id) and bool(request.lease_id))
            valid=bool(request.epoch and request.epoch==self.supervision.epoch and (current or restarted_idle))
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    async def execute_stop(self,handle):
        with self.lock:
            self.cancel('Supervisor: '+handle.request.reason)
            self.control.stop('Supervisor 정지 요청')
            future=Future()
            self.stop_waiters.append((handle,future,time.monotonic()))
        return await future

    def on_map(self,m):
        if m.header.frame_id != 'map':return
        q=m.info.origin.orientation
        with self.lock:
            updated = MapView(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z),list(m.data))
            if self.map is not None and updated.revision != self.map.revision:
                self.path = []
                self.clear_preview()
            self.map = updated

    def on_path(self, message):
        """Retain valid global plans, including plans received before goal acceptance."""
        with self.lock:
            if self.phase in ('stopping', 'cancelled', 'failed') or message.header.frame_id != 'map':
                return
            points = [[p.pose.position.x, p.pose.position.y] for p in message.poses]
            if any(not all(math.isfinite(v) for v in point) for point in points):
                return
            self.path = points
            self.path_at = time.monotonic()

    def on_scan(self,m):
        with self.lock:self.scan_at=time.monotonic()

    def on_odom(self,m):
        with self.lock:self.odom_at=time.monotonic()

    def on_velocity(self,m):
        with self.lock:
            self.velocity=(m.linear.x,m.linear.y,m.angular.z);self.velocity_at=time.monotonic()

    def readiness(self):
        if self.map is None:return False,'지도를 기다립니다'
        if time.monotonic()-self.pose_at>self.pose_timeout:return False,'초기 위치와 TF를 확인하세요'
        if time.monotonic()-self.scan_at>self.scan_timeout:return False,'LiDAR 입력이 오래됐습니다'
        if time.monotonic()-self.odom_at>self.odom_timeout:return False,'odom 입력이 오래됐습니다'
        if not self.client.server_is_ready():return False,'Nav2 서버를 기다립니다'
        # A required downstream subscriber prevents direct-to-base fallback.
        if self.node.count_subscribers('motion/cmd_vel')<1:return False,'안전 명령 경로를 기다립니다'
        if self.node.count_publishers('cmd_vel')!=1:return False,'최종 구동 명령 소유자를 확인하세요'
        available,reason=self.supervision.available()
        quality=getattr(self,'localization',{})
        if not available and quality.get('state') in ('unknown','unavailable'):
            reason='위치 정합 확인 필요: '+quality.get('reason','unknown')
        return available,reason

    def status(self):
        ready,why=self.readiness()
        return dict(phase=self.phase,reason=self.reason if self.phase!='idle' else why,
                    ready=ready,goal_id=self.goal_id,pose=self.pose,path=self.path,
                    preview_request_id=self.preview_id, preview_phase=self.preview_phase,
                    preview_path=self.preview_path, preview_error=self.preview_error,
                    preview_target=self.preview_target,
                    global_path_topic='/plan', global_path_points=len(self.path),
                    global_path_age_s=time.monotonic()-self.path_at if self.path else None,
                    last_input_failure=getattr(self,'last_input_failure',{}),
                    slip=getattr(self,'slip',{}),traction_recovery=getattr(self,'traction',{}),
                    localization=getattr(self,'localization',{}),
                    map_id=self.map.revision if self.map else None)

    def request(self,operation,data):
        if operation=='heartbeat':
            if data.get('goal_id')==self.goal_id:self.heartbeat=time.monotonic()
            return self.status()
        if operation=='cancel':
            self.clear_preview()
            self.cancel('사용자 정지');self.control.stop('사용자 정지');return self.status()
        if operation=='preview':return self.preview(data)
        if self.map is None:raise ValueError("지도를 기다립니다")
        goal=self.map.validate_goal(data)
        if operation=='initial-pose':
            if not self.control.stop_confirmed or not self.control.stopped():raise ValueError('먼저 로봇을 정지하세요')
            if self.phase in ('preparing','active','stopping'):raise ValueError('이동 중에는 초기 위치를 바꿀 수 없습니다')
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.node.get_clock().now().to_msg()
            msg.pose.pose.position.x,msg.pose.pose.position.y=goal[:2]
            msg.pose.pose.orientation.z=math.sin(goal[2]/2);msg.pose.pose.orientation.w=math.cos(goal[2]/2)
            msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.0685
            self.clear_preview()
            self.initial.publish(msg);return self.status()
        rid=data.get('request_id')
        if not isinstance(rid,str) or not 1<=len(rid)<=80:raise ValueError('잘못된 요청 번호')
        if rid in self.requests:
            if self.requests[rid]!=goal:raise ValueError('같은 요청 번호의 목표가 변경됐습니다')
            return self.status()
        ready,why=self.readiness()
        if not ready:raise ValueError(why)
        if self.phase not in ('idle','succeeded','cancelled','failed'):raise ValueError('먼저 현재 이동을 취소하세요')
        if len(self.requests)>=1024:raise ValueError('요청 기록 한도에 도달했습니다')
        self.clear_preview()
        self.requests[rid]=goal;self.epoch+=1;self.goal_id=str(uuid.uuid4());self.target=goal
        self.phase='preparing';self.reason='정지 및 주행 허가 확인 중';self.path=[]
        self.control.stop('자동 주행 전환');self.heartbeat=time.monotonic()
        self.control.automatic_client='navigation-'+self.goal_id
        self.supervision.acquire(self.goal_id);self.handle=None;self.pending=None;self.cancel_future=None;self.started_control=False
        self.velocity=(0.,0.,0.);self.velocity_at=0.
        return self.status()

    def clear_preview(self):
        """Invalidate old results before cancelling an in-flight read-only request."""
        self.preview_generation += 1
        if self.preview_handle is not None:
            self.preview_handle.cancel_goal_async()
        self.preview_handle=None;self.preview_path=[];self.preview_phase='idle';self.preview_target=None
        self.preview_error='';self.preview_id=''

    def preview(self, data):
        """Compute a global path without requesting a lease or enabling the base."""
        if self.map is None:
            raise ValueError('지도를 기다립니다')
        target=self.map.validate_goal(data)
        if self.phase in ('preparing','active','stopping'):
            raise ValueError('이동 중에는 먼저 취소한 뒤 새 목표를 선택하세요')
        request_id=data.get('request_id')
        if not isinstance(request_id,str) or not 1<=len(request_id)<=80:
            raise ValueError('잘못된 미리보기 요청 번호')
        self.clear_preview();self.path=[];self.preview_id=request_id
        if time.monotonic()-self.pose_at>self.pose_timeout:
            self.preview_phase='failed';self.preview_error='먼저 현재 위치를 설정하세요'
            return self.status()
        if not self.preview_client.server_is_ready():
            self.preview_phase='failed';self.preview_error='경로 계획 서버를 기다립니다'
            return self.status()
        self.preview_phase='planning'
        generation=self.preview_generation;map_id=self.map.revision
        goal=ComputePathToPose.Goal()
        goal.goal.header.frame_id='map';goal.goal.header.stamp=self.node.get_clock().now().to_msg()
        x,y,yaw=target;goal.goal.pose.position.x=x;goal.goal.pose.position.y=y
        goal.goal.pose.orientation.z=math.sin(yaw/2);goal.goal.pose.orientation.w=math.cos(yaw/2)
        goal.planner_id='GridBased';goal.use_start=False
        future=self.preview_client.send_goal_async(goal)
        future.add_done_callback(lambda f:self.preview_accepted(f,generation,map_id))
        return self.status()

    def preview_accepted(self, future, generation, map_id):
        with self.lock:
            try:
                handle=future.result()
            except Exception:
                if generation==self.preview_generation:
                    self.preview_phase='failed';self.preview_error='경로 계산 접수 실패'
                return
            if generation!=self.preview_generation or self.map is None or self.map.revision!=map_id:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.preview_phase='failed';self.preview_error='경로 계획 서버가 요청을 거부했습니다'
                return
            self.preview_handle=handle
            handle.get_result_async().add_done_callback(lambda f:self.preview_finished(f,generation,map_id))

    def preview_finished(self, future, generation, map_id):
        with self.lock:
            if generation!=self.preview_generation or self.map is None or self.map.revision!=map_id:
                return
            self.preview_handle=None
            try:
                result=future.result()
                path=result.result.path
                points=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
                valid=(result.status==GoalStatus.STATUS_SUCCEEDED and path.header.frame_id=='map'
                       and bool(points) and all(math.isfinite(v) for p in points for v in p))
                if valid:
                    pose=path.poses[-1].pose
                    q=pose.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    endpoint=[pose.position.x,pose.position.y,yaw]
                    valid=all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,*endpoint))
            except Exception:
                valid=False
            if valid:
                self.preview_path=points;self.preview_phase='ready';self.preview_error=''
                self.preview_target=endpoint
            else:
                self.preview_path=[];self.preview_phase='failed';self.preview_target=None
                self.preview_error='선택한 목표까지 경로를 찾지 못했습니다'

    def cancel(self,reason):
        if self.phase not in ('preparing','active','stopping'):return
        self.phase='stopping';self.reason=reason;self.outcome='cancelled';self.path=[]
        self.epoch+=1;self.control.stop(reason)
        if self.handle:self.cancel_future=self.handle.cancel_goal_async()

    def input_failure(self, now):
        inputs=(('odom','오도메트리',self.odom_at,self.odom_timeout),
                ('scan','LiDAR',self.scan_at,self.scan_timeout),
                ('pose','위치 추정/TF',self.pose_at,self.pose_timeout))
        ages={key:round(max(0.,now-stamp),3) for key,_,stamp,_ in inputs}
        for key,label,stamp,timeout in inputs:
            if now-stamp>timeout:
                return f'{label} 갱신 지연 ({now-stamp:.2f}초)',ages
        return '',ages

    def tick(self):
        now=time.monotonic()
        try:
            tf=self.buffer.lookup_transform('map','base_link',Time(),timeout=Duration(seconds=0.))
            age=(self.node.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
            if 0<=age<.5:
                q=tf.transform.rotation;p=tf.transform.translation
                self.pose=[p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)];self.pose_at=now
        except Exception:pass
        # Ask AMCL to process real scans while stationary; never refresh pose timestamps ourselves.
        if (now-self.nomotion_at>=.5 and self.nomotion.service_is_ready()
                and (self.nomotion_future is None or self.nomotion_future.done())):
            self.nomotion_at=now;self.nomotion_future=self.nomotion.call_async(Empty.Request())
        self.supervision.maintain()
        for handle,future,started in list(self.stop_waiters):
            stopped=(self.control.stop_confirmed and self.control.stopped()
                     and now-self.control.clock.received_at<=.1
                     and all(abs(v)<.05 for v in self.control.state.velocity_rad_s)
                     and (self.pending is None or self.pending.done())
                     and (self.cancel_future is None or self.cancel_future.done()))
            if stopped or now-started>5.:
                result=StopMission.Result(stopped=stopped,detail='MCU STOP acknowledged' if stopped else 'Stop unverified')
                if stopped:handle.succeed()
                else:handle.abort()
                future.set_result(result);self.stop_waiters.remove((handle,future,started))
        if self.phase=='stopping':
            if (self.control.stop_confirmed and self.control.stopped()
                    and now-self.control.clock.received_at<=.1
                    and all(abs(v)<.05 for v in self.control.state.velocity_rad_s)
                    and (self.pending is None or self.pending.done())
                    and (self.cancel_future is None or self.cancel_future.done())):
                self.supervision.release();self.control.automatic_client=None;self.phase=self.outcome
                self.reason='도착 완료' if self.phase=='succeeded' else self.reason+' · 정지 완료'
            return
        if self.phase not in ('preparing','active'):return
        auth,why=self.supervision.authorization()
        input_reason,ages=self.input_failure(now)
        if input_reason or auth==Authorization.DENIED:
            reason=input_reason or why or '주행 허가가 회수됐습니다'
            self.last_input_failure={'reason':reason,'input_ages_s':ages}
            self.control.event('navigation_input_failure',**self.last_input_failure)
            self.cancel(reason);return
        if auth!=Authorization.GRANTED:return
        if self.started_control and self.control.phase not in ('stopping_before_enable','enabling','moving'):
            self.cancel('구동 허가가 종료됐습니다');return
        if not self.started_control and not self.control.stop_confirmed:return
        if now-self.last_update<.07:return
        self.last_update=now
        velocity=self.velocity if self.phase=='active' and now-self.velocity_at<.2 else (0.,0.,0.)
        if not all(math.isfinite(v) for v in velocity):self.cancel('잘못된 Nav2 속도');return
        if self.control.forward_only and velocity[1]!=0:self.cancel('차단 프로필에서 횡이동 요청');return
        g=self.control.transport.geometry
        x,y,z=limit_wheel_velocity(velocity,g,self.max_wheel_speed)
        arm=(g.wheelbase+g.separation)/2
        wheels=[(x-y-arm*z)/g.radius,(x+y+arm*z)/g.radius,(x+y-arm*z)/g.radius,(x-y+arm*z)/g.radius]
        speed=max(.1,max(abs(v) for v in wheels));vector=[x/(g.radius*speed),y/(g.radius*speed),arm*z/(g.radius*speed)]
        # The joystick input normalization also bounds combined wheel commands.
        scale=max(1.,max(abs(v) for v in vector));vector=[v/scale for v in vector];speed*=scale
        self.sequence+=1
        try:
            self.control.request(dict(client='navigation-'+self.goal_id,sequence=self.sequence,action='update' if self.control.owner=='navigation-'+self.goal_id else 'press',vector=vector,speed=speed))
        except (ValueError,RuntimeError) as e:self.cancel(str(e));return
        self.started_control=True
        if self.phase=='preparing' and self.control.phase=='moving' and self.pending is None:
            msg=NavigateToPose.Goal();msg.pose.header.frame_id='map';msg.pose.header.stamp=self.node.get_clock().now().to_msg()
            x,y,a=self.target;msg.pose.pose.position.x=x;msg.pose.pose.position.y=y;msg.pose.pose.orientation.z=math.sin(a/2);msg.pose.pose.orientation.w=math.cos(a/2)
            epoch=self.epoch;self.pending=self.client.send_goal_async(msg)
            self.pending.add_done_callback(lambda f:self.accepted(f,epoch))

    def accepted(self,future,epoch):
        with self.lock:
            try:handle=future.result()
            except Exception as e:
                if epoch==self.epoch:self.cancel(str(e))
                return
            if epoch!=self.epoch:
                if handle.accepted:self.cancel_future=handle.cancel_goal_async()
                return
            if not handle.accepted:self.cancel('Nav2가 목표를 거부했습니다');return
            self.handle=handle;self.phase='active';self.reason='이동 중'
            handle.get_result_async().add_done_callback(lambda f:self.finished(f,epoch))

    def finished(self,future,epoch):
        with self.lock:
            if epoch!=self.epoch:return
            try:ok=future.result().status==GoalStatus.STATUS_SUCCEEDED
            except Exception:ok=False
            ok=bool(ok and self.pose and time.monotonic()-self.pose_at<self.pose_timeout)
            if ok:
                x,y,a=self.target
                ok=math.hypot(self.pose[0]-x,self.pose[1]-y)<=self.goal_distance and abs(math.atan2(math.sin(self.pose[2]-a),math.cos(self.pose[2]-a)))<=self.goal_angle
            self.control.stop('Nav2 완료');self.phase='stopping';self.outcome='succeeded' if ok else 'failed';self.reason='도착 정지 확인 중' if ok else '도착 조건 확인 실패'
            traction=getattr(self,'traction',{})
            if not ok and traction.get('reason') in ('slip_repeated_during_retry','slip_measurement_unavailable','longitudinal_retry_failed','stop_unverified'):
                self.reason='슬립 복구 중단: '+traction['reason']
