"""Check actual Humble namespace resolution for two simulator environments."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'tools/validation/isaac'))
from ros_scope import remappings
try:
    import rclpy
    from rclpy.action import ActionClient,ActionServer
    from control_msgs.action import FollowJointTrajectory
    from nav_msgs.msg import Odometry
except ImportError:
    rclpy=None


@unittest.skipIf(rclpy is None,'Run with ROS Humble sourced')
class ScopeTest(unittest.TestCase):
    def test_two_namespaces_keep_clock_shared_and_actions_separate(self):
        rclpy.init(args=[])
        nodes=[];clients=[];servers=[]
        try:
            for i in range(2):
                prefix=f'env_{i}'
                args=['--ros-args','-r',f'__ns:=/{prefix}']
                for source,destination in remappings(prefix):args.extend(['-r',f'{source}:={destination}'])
                node=rclpy.create_node(f'scope_test_{i}',cli_args=args);nodes.append(node)
                self.assertEqual(node.resolve_topic_name('/clock'),'/clock')
                for name in ['/tf','/wheel/odom','/scan','/joint_states','/mission/status']:
                    self.assertEqual(node.resolve_topic_name(name),f'/{prefix}'+name)
                self.assertEqual(node.resolve_service_name('controller_server/get_state'),f'/{prefix}/controller_server/get_state')
                node.create_publisher(Odometry,'wheel/odom',10)
                action='right_arm_controller/follow_joint_trajectory'
                servers.append(ActionServer(node,FollowJointTrajectory,action,lambda goal:FollowJointTrajectory.Result()))
                clients.append(ActionClient(node,FollowJointTrajectory,action))
            for client in clients:self.assertTrue(client.wait_for_server(timeout_sec=3))
            for node in nodes:
                self.assertEqual(node.count_publishers('/wheel/odom'),0)
                self.assertEqual(node.count_publishers(node.resolve_topic_name('/wheel/odom')),1)
            self.assertNotEqual(nodes[0].resolve_topic_name('cmd_vel'),nodes[1].resolve_topic_name('cmd_vel'))
        finally:
            for client in clients:client.destroy()
            for server in servers:server.destroy()
            for node in nodes:node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':unittest.main()
