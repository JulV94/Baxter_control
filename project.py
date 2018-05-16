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

        # joint angles in time
        self._waypoints_angles = self.inverse_kinematics()

    def format_accuracy(self):
        if not isinstance(self._config["accuracy"], (int, float)):
            self._config["accuracy"] = baxter_interface.settings.JOINT_ANGLE_TOLERANCE

    def inverse_kinematics(self):
        """
        Calculate the inverse kinematics and returns the result
        """
        results = list()
        for waypoint in self._config["waypoints"]:
            thetas = list()
            a1 = 370.82
    	    a2 = 374.42

            xprime = sqrt(waypoint["x"]*waypoint["x"] + waypoint["y"]*waypoint["y"])
            c3 = (xprime*xprime + waypoint["z"]*waypoint["z"] - a1*a1 - a2*a2)/(2*a1*a2)
            s3 = sqrt(1 - c3)

            thetas.append(atan2(waypoint["y"],waypoint["x"]) + pi)
            thetas.append(atan2(waypoint["z"],xprime) - atan2(a2*s3, a1 + a2*c3))
            thetas.append(0)
            thetas.append(atan2(s3,c3))
            thetas.append(0)
            thetas.append(-thetas[1] - thetas[2])
            thetas.append(0)

            results.append({"limb":waypoint["limb"], "values":dict(zip(self._robot.get_limb(waypoint["limb"]).joint_names(), thetas))})

        return results

    def get_name(self):
        return self._config["name"]

    def execute(self):
        """
        Execute the task on the robot
        """
        rospy.sleep(1.0)

        rospy.loginfo("Task Started")

        # Set joint position speed ratio for execution
        self._robot.get_limb("left").set_joint_position_speed(self._config["speed"])
        self._robot.get_limb("right").set_joint_position_speed(self._config["speed"])

        # Play the waypoints
        for angles in self._waypoints_angles:
            if rospy.is_shutdown():
                break
            self._robot.get_limb(angles["limb"]).move_to_joint_positions(angles["values"], timeout=20.0, threshold=self._config["accuracy"])

        # Sleep for a few seconds
        rospy.sleep(3.0)

        # Set joint position speed back to default
        self._robot.get_limb("left").set_joint_position_speed(0.3)
        self._robot.get_limb("right").set_joint_position_speed(0.3)

class BaxterRobot(object):
    def __init__(self):
        # Init the limbs of the robot
        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")

        # Init the robot
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def get_limb(self, name):
        if name == "left":
            return self._limb_left
        elif name == "right":
            return self._limb_right
        rospy.logerr("%s does not name any limb", name)
        exit(1)

    def shutdown_robot(self):
        print("\nExiting code...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

def main():
    """RSDK Joint Position Waypoints

    Calculate some waypoints from the inverse kinematics then move
    """
    print("Initializing node... ")
    rospy.init_node("inverse_kinematics_task")
    robot = BaxterRobot();

    # Register clean shutdown
    rospy.on_shutdown(robot.shutdown_robot)

    task = BaxterTask(robot, "src/baxter_examples/scripts/task.json")

    # Begin example program
    print("Running task " + task.get_name())
    task.execute()

if __name__ == '__main__':
    main()
