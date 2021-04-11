import rospy
import smach
from tf2_msgs.msg import TFMessage
from math import degrees
from tf.transformations import euler_from_quaternion
import tf2_ros

class GetCurrentPos(smach.State):
   

   def __init__(self):
      smach.State.__init__(self, outcomes=['success','fail'],input_keys=[], output_keys=['curr_pos'])
      self.tfBuffer = tf2_ros.Buffer()
      self.listener = tf2_ros.TransformListener(self.tfBuffer)
      

   def execute(self, userdata):
      rate = rospy.Rate(10.0)
      while not rospy.is_shutdown():
         try:
           point = self.tfBuffer.lookup_transform('map', 'pioneer/base_link', rospy.Time())
           break
         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
           rate.sleep()
           continue
    #   while(not(odom_msg and base_link_msg)):
    #       msg = rospy.wait_for_message("/tf", TFMessage)
          
    #       point = msg.transforms[0] 
    #       name = point.child_frame_id

    #       if name == "pioneer/odom":
    #           odom_msg = point
    #       elif name == "/pioneer/base_link":
    #           base_link_msg = point
      
      robot_tf = point.transform
      
      robot_pos_translation = robot_tf.translation

      curr_pos = [robot_pos_translation.x, robot_pos_translation.y]
      
      robot_pos_rotation = point.transform.rotation
      robot_pos_rotation = [robot_pos_rotation.x, robot_pos_rotation.y, robot_pos_rotation.z, robot_pos_rotation.w]

      robot_pos_rotation_degress = degrees(euler_from_quaternion(robot_pos_rotation)[-1])
      
      curr_pos.append(robot_pos_rotation_degress)

      print('Current position: {}'.format(curr_pos))
      if userdata:
          userdata.curr_pos = curr_pos

      return 'success'

if __name__ == '__main__':
   rospy.init_node('check_door')
   node = GetCurrentPos()
   node.execute(None)
