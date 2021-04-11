import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
from std_msgs.msg import String
import smach

class NavGoalB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'],
                            input_keys=['struct', 'location'],
                            output_keys=[])

        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.locations = {
                'shelf': [2.88, -2.7, 0.0],
                'garage': [5.82, -11.5, 0.0],
                'kitchen': [2, -2.5, 0],
                'dining': [2.51, -6.96, 0],
                'living': [5.39, -7.87, -1.0],
                'corridor': [1.44, 0.94, -1.0],
                'bedroom': [5.8, -2.15, 0]}


    def setupGoal(self, loc):
        x, y, yaw = loc

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map' # Note: the frame_id must be map
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        angle = radians(yaw) # angles are expressed in radians
        quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
        goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
        return goal

    def execute(self, userdata):
#        if userdata.struct['command'] == 'follow':
#            goal = self.setupGoal(userdata.location)
#        else:
#            goal = self.setupGoal(self.locations[userdata.location])
        goal = self.setupGoal(self.locations['dining'])

        self._client.send_goal(goal)
        self._client.wait_for_result()
        nav_res = self._client.get_result()

        if nav_res:
            return 'success'
            #return userdata.mode
        return 'fail'

if __name__ == '__main__':
    rospy.init_node('RosNavGoalPublisher')
    rngp = RosNavGoalPublisher()
    rospy.loginfo('Node initialized...')
    rospy.spin()
