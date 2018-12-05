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

import argparse
import math
import random
import struct
import sys
import copy
import os
import subprocess, signal

import rospy
import rospkg

import rospy

from std_msgs.msg import (
    UInt16,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


import baxter_interface

from baxter_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.Limb("left")
        self._limb_name = "left"
        self._verbose = True
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._gripper = baxter_interface.Gripper("left")
        self._right_joint_names = self._right_arm.joint_names()
        self._gripper = baxter_interface.Gripper("left")
        ns = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)
    def move_to_start(self, start_angles=None):
		print("Moving the {0} arm to start pose...".format(self._limb_name))
		if not start_angles:
			start_angles = dict(zip(self._joint_names, [0]*7))
		self._guarded_move_to_joint_position(start_angles)
		self.gripper_open()
		rospy.sleep(1.0)
		print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._left_arm.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + 0.15
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose, filename):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        #start to record
        rosbag_process = start_rosbag_recording(filename)
        self._approach(pose)
        return rosbag_process
        
       

    def place(self, pose, rosbag_process):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # stop rosbag recording
        stop_rosbag_recording(rosbag_process)
        self._approach(pose)
        
    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        #self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def wobble(self, rosbag_process, pose):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
            period_factor = 0.5#random.uniform(0.3, 0.5)
            amplitude_factor = 0.45#random.uniform(0.1, 0.2)

            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in self._right_joint_names]

        def make_cmd(joint_names, elapsed):
            return dict([(joint, v_funcs[i](elapsed))
                         for i, joint in enumerate(joint_names)])

        print("Wobbling. Press Ctrl-C to stop...")
        num_= 5000;
        while (num_>0):
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._left_joint_names, elapsed)
            num_= num_-1
            cmd['left_w0'] = 0.0
            cmd['left_e0'] = 0.0
            cmd['left_s0'] = 0.0
            
            print(cmd)
            
            self._left_arm.set_joint_velocities(cmd)
            
            #cmd = make_cmd(self._right_joint_names, elapsed)
            #print(cmd)
            #self._right_arm.set_joint_velocities(cmd)
            rate.sleep()
        self.place(pose, rosbag_process)

def load_gazebo_models(box_no = 4, table_pose=Pose(position=Point(x=1.0, y= 0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    script_path = os.path.dirname(os.path.abspath(__file__)) 
    model_path = script_path[:-8]+"/models/"
    
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    
    # Load Blocks URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
  
   # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
   
   # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def load_gazebo_block(box_no = 4, block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    script_path = os.path.dirname(os.path.abspath(__file__)) 
    model_path = script_path[:-8]+"/models/"
     
    # Load Blocks URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
  
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_block():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def start_rosbag_recording(filename):
    # find the directory to save to
    rospy.loginfo(rospy.get_name() + ' start')
    script_path = os.path.dirname(os.path.abspath(__file__))
    rosbagfile_dir = script_path[:-8]+"/rosbagfiles/"
    
    #  modify the rosbag process with prof Jivko
    rosbag_process = subprocess.Popen('rosbag record -o {} /robot/joint_states'.format(filename), stdin=subprocess.PIPE, shell=True, cwd= rosbagfile_dir)
    return rosbag_process
    
def stop_rosbag_recording(p):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    rospy.loginfo(p.pid)
    
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()  # we wait for children to terminate
    
    rospy.loginfo("I'm done")

def main():
    rospy.init_node("rsdk_joint_velocity_wobbler")
    
    #parse argument
    myargv = rospy.myargv(argv=sys.argv)
    filename = "baxter_wobbler__model"+ str(myargv[1])+"_"
    num_of_run = int(myargv[2])
    
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models(myargv[1])
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
                             
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    
    block_pose = Pose(position= Point(x=0.7, y=0.15, z=-0.129), orientation=overhead_orientation)
    wobbler = Wobbler()
    
    for x in range(0,num_of_run):
        if(not rospy.is_shutdown()):
			wobbler.move_to_start(starting_joint_angles)
			rosbag_process =  wobbler.pick(block_pose, filename)
			rospy.on_shutdown(wobbler.clean_shutdown)
			wobbler.wobble(rosbag_process, block_pose)
			wobbler.move_to_start(starting_joint_angles)
			delete_gazebo_block()
			load_gazebo_block(myargv[1])
            
        else:
            break
    
    print("Done.")

if __name__ == '__main__':
    main()
