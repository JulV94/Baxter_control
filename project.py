#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Waypoints Tasks
"""
#import sys
import rospy
import baxter_interface
import json
from math import atan2, sqrt, pi

class BaxterTask(object):
    def __init__(self, robot, config_file):
        # Store robot instance
        self._robot = robot;

        # Config for the Task
        with open(config_file) as f:
            self._config = json.load(f)

        # Put default accuracy if not well specified
        self.format_accuracy()

        # Change points to absolute positions
        self.relToAbsolutePosition(self._config["waypoints"])

        # joint angles in time
        self._grippoints_angles = self.inverse_kinematics(self._config["grippoints"])
        self._initpoints_angles = self.inverse_kinematics(self._config["initpoints"])
        self._waypoints_angles = self.inverse_kinematics(self._config["waypoints"])

    def format_accuracy(self):
        if not isinstance(self._config["accuracy"], (int, float)):
            self._config["accuracy"] = baxter_interface.settings.JOINT_ANGLE_TOLERANCE

    def relToAbsolutePosition(self, points):
        for point in points:
            for initpoint in self._config["initpoints"]:
                if point["limb"] == initpoint["limb"]:
                    point["x"] += initpoint["x"]
                    point["y"] += initpoint["y"]
                    point["z"] += initpoint["z"]

    def inverse_kinematics(self, points):
        """
        Calculate the inverse kinematics and returns the result
        """
        results = list()
        for point in points:
            thetas = list()
            a1 = 0.37082
    	    a2 = 0.37442

            xprime = sqrt(point["x"]*point["x"] + point["y"]*point["y"]) -0.069
            zprime = point["z"]-0.22952+0.27035
            c3 = (xprime*xprime + zprime*zprime - a1*a1 - a2*a2)/(2*a1*a2)
            s3 = sqrt(1 - c3*c3)

            if point["limb"] == "left":
                angle = -pi/4
            else:
                angle = pi/4

            thetas.append(atan2(point["y"],point["x"]) + angle)
            thetas.append(atan2(zprime,xprime) - atan2(a2*s3, a1 + a2*c3))
            thetas.append(0)
            thetas.append(atan2(s3,c3))
            thetas.append(0)
            thetas.append(pi/2 - thetas[1] - thetas[3])
            thetas.append(0)


            results.append({"limb":point["limb"], "values":dict(zip(self._robot.get_limb(point["limb"]).joint_names(), thetas))})

        return results

    def get_name(self):
        return self._config["name"]

    def waitingToGrip(self):
        self.gotoPoints(self._grippoints_angles)
        self._robot.gripperControl()

    def gotoPoints(self, points):
        # Play the waypoints
        for point in points:
            if rospy.is_shutdown():
                break
            rospy.loginfo("Generated angles are : " + str(point["values"]))
            self._robot.get_limb(point["limb"]).move_to_joint_positions(point["values"], timeout=20.0, threshold=self._config["accuracy"])

    def execute(self):
        """
        Execute the task on the robot
        """
        rospy.sleep(1.0)

        rospy.loginfo("Task " + self._config["name"] + "Started")

        # Set joint position speed ratio for execution
        self._robot.get_limb("left").set_joint_position_speed(self._config["speed"])
        self._robot.get_limb("right").set_joint_position_speed(self._config["speed"])

        self.waitingToGrip()
        self.gotoPoints(self._initpoints_angles)
        self._robot.waitForStateChange(2)
        self.gotoPoints(self._waypoints_angles)

        # Sleep for a few seconds
        rospy.sleep(3.0)

        # Set joint position speed back to default
        self._robot.get_limb("left").set_joint_position_speed(0.3)
        self._robot.get_limb("right").set_joint_position_speed(0.3)

class BaxterRobot(object):
    def __init__(self):
        # Init the limbs of the robot
        rospy.init_node("inverse_kinematics_task")
        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")

        # Init grippers of the robot
        self._gripper_left = baxter_interface.Gripper("left", baxter_interface.CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", baxter_interface.CHECK_VERSION)

        self._gripper_left.calibrate()
        self._gripper_right.calibrate()

        # Create Navigator I/O
        self._navigator_left = baxter_interface.Navigator("left")
        self._navigator_right = baxter_interface.Navigator("right")

        # State of the robot
        self.state = 0;

        # Init the robot
        rospy.loginfo("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self._rs.enable()

    def get_limb(self, name):
        if name == "left":
            return self._limb_left
        elif name == "right":
            return self._limb_right
        rospy.logerr("%s does not name any limb", name)
        exit(1)

    def connectButtonState(self):
        self._navigator_left.button2_changed.connect(self.incrementState)
        self._navigator_right.button2_changed.connect(self.incrementState)

    def disconnectButtonState(self):
        self._navigator_left.button2_changed.disconnect(self.incrementState)
        self._navigator_right.button2_changed.disconnect(self.incrementState)

    def incrementState(self, value):
        if value:
            self.state += 1

    def resetState(self):
        self.state = 0

    def gripperControl(self):
        self._navigator_left.wheel_changed.connect(self.leftWheelMoved)
        self._navigator_right.wheel_changed.connect(self.rightWheelMoved)
        self.waitForStateChange(1)
        self._navigator_left.wheel_changed.disconnect(self.leftWheelMoved)
        self._navigator_right.wheel_changed.disconnect(self.rightWheelMoved)

    def leftWheelMoved(self, value):
        self._gripper_left.command_position(self._navigator_left.wheel/2.55)

    def rightWheelMoved(self, value):
        self._gripper_right.command_position(self._navigator_right.wheel/2.55)

    def waitForStateChange(self, state):
        self.connectButtonState()
        while (self.state != state) and not rospy.is_shutdown():
            rospy.sleep(1.0)
        self.disconnectButtonState()

    def shutdown_robot(self):
        rospy.loginfo("\nExiting code...")
        if not self._init_state:
            rospy.loginfo("Disabling robot...")
            self._rs.disable()
        return True

def main():
    """RSDK Joint Position Waypoints

    Calculate some waypoints from the inverse kinematics then move
    """
    print("Initializing node... ")
    robot = BaxterRobot();

    # Register clean shutdown
    rospy.on_shutdown(robot.shutdown_robot)

    task = BaxterTask(robot, "src/baxter_examples/scripts/task.json")

    # Begin example program
    rospy.loginfo("Running task " + task.get_name())
    task.execute()

if __name__ == '__main__':
    main()
