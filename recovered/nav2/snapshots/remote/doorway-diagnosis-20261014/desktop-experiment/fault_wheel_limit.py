"""Explicit Gazebo-only command-hold fault injection for recovery validation."""
import os,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent/'laptop-runtime'))
from wheel_limit_node import WheelLimit
import rclpy
class HeldWheelLimit(WheelLimit):
 def __init__(self):
  super().__init__();self.begin,self.end=map(float,os.environ['MPPI_TEST_HOLD'].split(':'))
  assert 0<=self.begin<self.end
 def emit(self,output,scale,reason):
  if self.begin<=self.now()<self.end:output=(0.,0.,0.);scale=0.;reason='TEST_COMMAND_HOLD'
  return super().emit(output,scale,reason)
if __name__=='__main__':
 rclpy.init();node=HeldWheelLimit()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.try_shutdown()
