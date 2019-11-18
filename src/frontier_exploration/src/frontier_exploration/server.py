#!/usr/bin/env python

import rospy
import actionlib
import nav_msgs.msg
import fronter_exploration
import move_base_msgs
from frontier_costmap import FrontierCostmap, Frontier


class ExploreServer():
    '''
    Explore Server is an action server to explore 
    '''
    def __init__(self):
        # The map that we are searching across
        self._costmap = FrontierCostmap(nav_msgs.msg.OccupancyGrid())
        # Initialize the action server
        self._explore_action_server = actionlib.SimpleActionServer(
            "explore_frontiers",
            fronter_exploration.msg.ExploreFrontiers,
            execute_cb=self.exploreActionCb,
            auto_start=False)

        # Initialize the move base client
        self._movebase_client = actionlib.SimpleActionClient(
            'move_base', move_base_msgs.msg.MoveBaseAction)

        # Initialize subscriber to the map topic
        rospy.Subscriber(rospy.get_param('~map_topic', '/map'), self.onMap)

        # Initialize subscriber to the map topic
        self._odom = None
        rospy.Subscriber(rospy.get_param('~odom_topic', '/odom'), self.onOdom)

    # Executes a new exploration task
    def exploreActionCb(self, goal):
        #TODO finish this method
        while True:
            robot_grid_pos = (self._odom.pose.pose.position.x,
                              self._odom.pose.pose.position.y)
            frontiers = self._costmap.findFrontiers(*robot_grid_pos)
            if (len(frontiers) == 0):
                break
            closest_frontier = min(
                frontiers,
                key=lambda f: self.dist(f.getMiddle(), robot_grid_pos))

            #todo convert to world position
            #send move base task
        self._explore_action_server.set_succeeded()

    # Handles new map information reveived on the configured map topic
    # OccupancyGrid -> void
    def onMap(self, map_msg):
        self._costmap = FrontierCostmap(map_msg)

    def onOdom(self, odom_msg):
        self._odom = odom_msg

    def dist(self, p1, p2):
        pass