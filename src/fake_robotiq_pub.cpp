/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Alberto Romay, TU Darmstadt ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// This is a dummy node which will publish fake Robotiq hand joint states so we
// can test in simulation.  We don't need this node if we have a working Gazebo plugin.

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include "robotiq_s_model_control/SModel_robot_output.h"

#define ROBOTIQ_NUM_JOINTS 11

#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPREAD_RAD     0.28   //Radians range of the spread fingers
#define BYTE_TO_SPR    (SPREAD_RAD/255.0)
#define SPR_TO_BYTE    (1.0/BYTE_TO_SPR)
#define SPR_ZERO       (BYTE_TO_SPR * 137.0)
#define PER_TO_BYTE    2.55

class RobotiqRepub
{
    public:
    ros::NodeHandle n;
    ros::Publisher  robotiq_sim_states_pub;
    ros::Subscriber grasp_controller_sub;

    sensor_msgs::JointState     hand_joints_;

    void robotiqInit(std::string hand_side, std::string hand_name);
    void robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr & msg);
};

// Update hand values from flor_grasp_controller publishing
void RobotiqRepub::robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg)
{
    this->hand_joints_.position[0] = msg->rPRA * 0.004784314;
    this->hand_joints_.position[1] = msg->rPRB * 0.004784314;
    this->hand_joints_.position[2] = msg->rPRC * 0.004784314;
    this->hand_joints_.position[3] = msg->rPRS *  BYTE_TO_SPR - SPR_ZERO;
    this->hand_joints_.position[4] = msg->rPRS * -BYTE_TO_SPR + SPR_ZERO;
}

// Initialize hand values to 0
void RobotiqRepub::robotiqInit(std::string hand_side,std::string hand_name)
{
    hand_joints_.name.resize(ROBOTIQ_NUM_JOINTS);
    hand_joints_.position.resize(ROBOTIQ_NUM_JOINTS);
    hand_joints_.velocity.resize(ROBOTIQ_NUM_JOINTS);
    hand_joints_.effort.resize(ROBOTIQ_NUM_JOINTS);

    for(int i = 0; i < ROBOTIQ_NUM_JOINTS; i++)
    {
        hand_joints_.position[i] = 0;
        hand_joints_.velocity[i] = 0;
        hand_joints_.effort[i]   = 0;
    }

    hand_joints_.name[0]  = hand_side + "_f0_j1";
    hand_joints_.name[1]  = hand_side + "_f1_j1";
    hand_joints_.name[2]  = hand_side + "_f2_j1";
    hand_joints_.name[3]  = hand_side + "_f1_j0";
    hand_joints_.name[4]  = hand_side + "_f2_j0";
    hand_joints_.name[5]  = hand_side + "_f0_j2";
    hand_joints_.name[6]  = hand_side + "_f1_j2";
    hand_joints_.name[7]  = hand_side + "_f2_j2";
    hand_joints_.name[8]  = hand_side + "_f0_j3";
    hand_joints_.name[9]  = hand_side + "_f1_j3";
    hand_joints_.name[10] = hand_side + "_f2_j3";

    grasp_controller_sub   = n.subscribe("/robotiq_hands/"+hand_name+"/SModelRobotOutput", 1, &RobotiqRepub::robotiqRepubCallback, this);
    robotiq_sim_states_pub = n.advertise<sensor_msgs::JointState>("/robotiq_hands/"+hand_name+"/joint_states",1);

}


int main(int argc, char** argv)
{
    // Create publisher
    ros::init(argc, argv, "fake_robotiq_pub");

    std::string hand_name_;
    std::string hand_side_;

    ros::NodeHandle("~").getParam("hand_side", hand_side_);
    ros::NodeHandle("~").getParam("hand_name", hand_name_);
    ROS_INFO("hand type: %s", hand_side_.c_str());

    //Right hand class
    RobotiqRepub myrepubRight;
    myrepubRight.robotiqInit(hand_name_, hand_side_);

    ros::Publisher robotiq_repub;
    robotiq_repub = myrepubRight.n.advertise<sensor_msgs::JointState>("/joint_states",1); //Only created once, used by both left and right

    while(ros::ok())
    {
        myrepubRight.hand_joints_.header.stamp = ros::Time::now();
        robotiq_repub.publish(myrepubRight.hand_joints_);
        myrepubRight.robotiq_sim_states_pub.publish(myrepubRight.hand_joints_);

        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
