#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import actionlib
import rospy

from sagittarius_object_color_detector.msg import (
    SGRCtrlAction,
    SGRCtrlFeedback,
    SGRCtrlResult,
)


ACTION_NAMES = {
    0: "XYZ",
    1: "XYZ_RPY",
    2: "DEFINE_STAY",
    3: "DEFINE_SAVE",
    4: "PICK_XYZ",
    5: "PICK_XYZ_RPY",
    6: "PUT_XYZ",
    7: "PUT_XYZ_RPY",
}

GRASP_NAMES = {
    0: "NONE",
    1: "OPEN",
    2: "CLOSE",
}


class MockSGRCtrlServer:
    def __init__(self):
        rospy.init_node("mock_sgr_ctrl_server")
        robot_name = rospy.get_param("~robot_name", "sgr532")
        action_name = f"/{robot_name}/sgr_ctrl"

        self.server = actionlib.SimpleActionServer(
            action_name,
            SGRCtrlAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[MockSGR] dry-run action server started: %s", action_name)

    def execute(self, goal):
        action = ACTION_NAMES.get(goal.action_type, str(goal.action_type))
        grasp = GRASP_NAMES.get(goal.grasp_type, str(goal.grasp_type))
        rospy.logwarn(
            "[MockSGR] goal action=%s grasp=%s pos=(%.3f, %.3f, %.3f) "
            "rpy=(%.3f, %.3f, %.3f)",
            action,
            grasp,
            goal.pos_x,
            goal.pos_y,
            goal.pos_z,
            goal.pos_roll,
            goal.pos_pitch,
            goal.pos_yaw,
        )

        feedback = SGRCtrlFeedback()
        feedback.step = SGRCtrlFeedback.EXEC_POSITION
        self.server.publish_feedback(feedback)

        result = SGRCtrlResult()
        result.result = SGRCtrlResult.SUCCESS
        self.server.set_succeeded(result)


if __name__ == "__main__":
    MockSGRCtrlServer()
    rospy.spin()
