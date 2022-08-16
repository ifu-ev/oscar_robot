#!/usr/bin/env python
import tf
import math

import rospy
# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#######################################################################
# # ADD YOUR CODE HERE, WAYPOINTS       
# # FORMAT: [(x-coordinate,y-coordinate),(x-coordinate,y-coordinate)] <- 2 Waypoints in meter
# # x: Straight/Backwards 
# # y: Left/Right
WAYPOINTS_LINE = [(0.5,0),(1.0,0),(1.5,0)]
WAYPOINTS_SQUARE = [(1,0),(1,1),(0,1),(0,0),(1,0)]
WAYPOINTS_YOUR_OWN = []
#######################################################################


def movebase_client(curr_x, curr_y, next_x, next_y):
    """sends goal to move_base server"""
    # get angle to next waypoint
    angle_to_goal = math.atan2(next_y - curr_y, next_x - curr_x)
    # transform angle to quaternion
    q_angle = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal, axes='sxyz')
    # create Quaternion ROS message and fill it with quaternion data
    q = Quaternion(*q_angle)
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Waits until the action server has started up and started listening for goals.
    result = client.wait_for_server()
    if not result:
        rospy.logerr("Action server not ready!")
        rospy.signal_shutdown("Action server not ready!")
        return False
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    # Set frame_id of goal to our map frame
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose = Pose(Point(curr_x, curr_y, 0.000), q)
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the actions
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return False
    else:
    # Result of executing the action
        return client.get_result() 

def waypoint_menu():
    print("####################################")
    print("Which waypoint do you want to go to?")
    print("(1) Line")
    print("(2) Square")
    print("(3) Your own")
    choice = int(input("Enter your choice: "))
    if 0 < choice < 4:
        return choice
    else:
        print("Invalid choice")
        waypoint_menu()

def waypoint_menu_own():
    wp_num = 1
    print("########################################")
    print("How many waypoints do you want to go to?")
    choice = input("Enter your choice: ")
    if int(choice) > 0:
        for i in range(int(choice)):
            print("########################################")
            print("Enter the x and y coordinates of waypoint {}: ".format(wp_num))
            x = input("x: ")
            y = input("y: ")
            WAYPOINTS_YOUR_OWN.append((float(x), float(y)))
            wp_num += 1
        return WAYPOINTS_YOUR_OWN
    else:
        print("Invalid choice")
        waypoint_menu_own()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        # Print some info into the ros log system
        rospy.loginfo("Starting move_base_client_py...")
        # Print waypoint menu
        choice = waypoint_menu()
        # select choice from menu
        if choice == 1:
            waypoints = WAYPOINTS_LINE
        elif choice == 2:
            waypoints = WAYPOINTS_SQUARE
        else:
            waypoints = waypoint_menu_own()
        # go through waypoints
        for x in range (0, len(waypoints)):
            if x+1==len(waypoints):
                # send goal to last waypoint and exit 
                result = movebase_client(waypoints[x][0],waypoints[x][1], waypoints[0][0], waypoints[0][1])
                exit()
            else:
                # send goal to next waypoint
                result = movebase_client(waypoints[x][0],waypoints[x][1],waypoints[x+1][0],waypoints[x+1][1])     
            if result:
                rospy.loginfo("Goal execution done!")       
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
