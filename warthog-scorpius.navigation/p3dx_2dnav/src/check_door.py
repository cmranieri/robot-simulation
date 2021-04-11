import rospy
import smach
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin

class CheckDoor(smach.State):
   
   DETECTION_ANGLE_RANGE = (10 * pi) / 180 # rad
   DOOR_TRESHOLD = 1

   def __init__(self):
      smach.State.__init__(self, outcomes=['close','open'],input_keys=[], output_keys=[])
      

   def execute(self, userdata):
      print('antes')
      msg = rospy.wait_for_message("/scan", LaserScan)
      print('depois')
      self.angle_min = msg.angle_min
      self.angle_increment = msg.angle_increment
      self.range_max = msg.range_max
      self.range_min = msg.range_min
      self.laser = msg.ranges

      state = self.process_laser_msg()
      print(state)
      return state

   def get_index_by_angle(self, desire_angle):
      return int((desire_angle - self.angle_min) / self.angle_increment)

   def is_laser_range_valid(self, laser_range):
       # Ignore ground detection
       return laser_range and laser_range > self.range_min

   def process_laser_msg(self):
      min_angle_index = self.get_index_by_angle(-self.DETECTION_ANGLE_RANGE / 2)
      max_angle_index = self.get_index_by_angle(self.DETECTION_ANGLE_RANGE / 2)

      closest_range = self.range_max
      closest_index = None

      for index in range(min_angle_index, max_angle_index):
         laser_range = self.laser[index]
         if self.is_laser_range_valid(laser_range) and laser_range <= self.DOOR_TRESHOLD:
            return 'close'
      
      return 'open'

if __name__ == '__main__':
   rospy.init_node('check_door')
   check_door = CheckDoor()
   check_door.execute(None)
