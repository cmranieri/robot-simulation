import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians
import smach

LOCATIONS = {
	'inspection': [4.13274, 5.80582, 90],
	'entrance': [0.0827904, 0.00226355, 0],
	'exit': [-0.293018, 7.99665, 0],
	'living room': [2.13478, 1.59524, 270],
	'meet': [3.66302, 2.66513, 190],
	'kitchen': [4.13274, 5.80582, 90],
	'bedroom': [8.82156, 1.11236, 180],
	'office': [7.1097, 6.4337, 270],
	'shelf': [1.57168, 6.10009, 190],
	'close shelf': [1.25398, 6.07773, 190],
	'small table': [2.52329, 1.48136, 15],
	'trash kitchen': [1.8055, 4.94418, 190],
	'trash middle': [1.8055, 4.94418, 0],
	'trash zone': [1.28533, 7.84911, 90],
	'trash zone2': [1.28533, 7.44911, 90],
	'trash office': [7.1097, 6.4337, 0],
	'guest1': [2.63533, 0.816867, 320],
	'guest2': [2.63533, 0.816867, 320],
	'guest3': [2.63533, 0.816867, 320],
	'guest4': [2.63533, 0.816867, 320],
	'guest5': [2.63533, 0.816867, 320],
	
	'where_information': [1.53748, 5.97867, 270],
	'where_big_trash': [1.53748, 5.97867, 240],
	'where_exit': [1.64086, 7.37237, 150],
	'where_entrance' : [2.02391, 1.00961, 225],
	'where_refrigerator': [3.53785, 7.53271, 35],
	'where_small_refrigerator': [3.53785, 7.53271, 325],
	'where_wooden_shelf': [1.97441, 7.15065, 90],
	'where_microwave': [4.01542, 6.09244, 0],
	'where_cabinet': [3.95369, 5.37745, 345],
	'where_living_room': [1.74998, 2.87718, 270],
	'where_tv': [1.93408, 2.02563, 180],
	'where_black_sofa': [1.59154, 3.36706, 0],
	'where_orange_sofa': [1.82735, 0.16286, 0],
	'where_center_table': [1.93408, 2.02563, 0],
	'where_phone': [1.35994, 3.56486, 225],
	'where_plant': [4.27615, 4,27192, 315],
	
	
	'where_office': [7.21163, 5.01096, 180],
	'where_computer_desk': [7.21163, 5.01096, 45],
	'where_writing_desk': [7.21163, 5.01096, 225],
	'where_bookshelf': [6.84149, 7.33228, 90],
	
	'where_small_trash': [7.19382, 7.19078, 0],
	
	'where_bedroom': [7.80871, 0.914517, 180],
	'where_bed': [7.80871, 0.914517, 90],
	'where_armchair': [6.25586, 2.22102, 90],
	
}


class FindLocation(smach.State):
    def __init__(self,location=None):
        smach.State.__init__(self, outcomes=['fail', 'success'],
                             input_keys=['name_location', 'curr_pos'],
                             output_keys=['locations', 'position'])
        self.location = location

    def execute(self, userdata):
        name_location = self.location or userdata.name_location
        if name_location == 'banana':
            position = userdata.curr_pos
        else:
            position = LOCATIONS.get(name_location)
        print(position)
        if position:
            userdata.position = position
            return 'success'
        return 'fail'


class FindRotateLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail', 'success'],
                             input_keys=['curr_pos'],
                             output_keys=['position'])

    def execute(self, userdata):
        current_location = userdata.curr_pos
        print(current_location)
        current_angle = current_location[2]
        rotated_angle = current_angle + 180
        userdata.position = [current_location[0], current_location[1], rotated_angle]
        print([current_location[0], current_location[1], rotated_angle])
        return 'success'
        

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['position'],
                             output_keys=[])

        self._client = actionlib.SimpleActionClient('move_base',
                                                    MoveBaseAction)

    def execute(self, userdata):
        goal_position = userdata.position

        if not goal_position:
            return 'fail'

        goal = self.setupGoal(userdata.position)

        self._client.wait_for_server()
        self._client.send_goal(goal)
        self._client.wait_for_result()
        nav_result = self._client.get_result()

        if nav_result:
            return 'success'
        return 'fail'

    def setupGoal(self, position):
        x, y, yaw = position

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        angle = radians(yaw)  # angles are expressed in radians
        quaternion = quaternion_from_euler(0.0, 0.0, angle)  # roll, pitch, yaw
        goal.target_pose.pose.orientation = Quaternion(*quaternion.tolist())
        return goal

# def log(self, str2print):
#     output = "[" + time.strftime("%H:%M:%S", time.gmtime()) + "] -> " + str2print
#     print output
#     fh = open("log.txt", "a")
#     fh.write(output+"\n\n")
#     fh.close
#
