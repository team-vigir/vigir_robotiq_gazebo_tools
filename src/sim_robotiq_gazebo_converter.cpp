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
#include "atlas_msgs/SModelRobotOutput.h"
#include "ros/ros.h"
#include "robotiq_s_model_control/SModel_robot_output.h"
#include "robotiq_s_model_control/SModel_robot_input.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define ROBOTIQ_NUM_FAKE_JOINTS 28

#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPREAD_RAD     0.28   //Radians range of the spread fingers
#define BYTE_TO_SPR    (SPREAD_RAD/255.0)
#define SPR_TO_BYTE    (1/BYTE_TO_SPR)
#define SPR_ZERO       (BYTE_TO_SPR * 137)
#define PER_TO_BYTE    2.55

class RobotiqRepub
{
    public:
    ros::NodeHandle n;
    ros::Publisher  robotiq_sim_input_pub;
    ros::Publisher  robotiq_sim_commands_pub;
    ros::Subscriber robotiq_sim_output_sub;
    ros::Subscriber robotiq_sim_states_sub;

    robotiq_s_model_control::SModel_robot_input robotiq_input_;

    atlas_msgs::SModelRobotOutput               atlas_output_;


    void robotiqInit(std::string hand_side, std::string hand_name);
    void robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr & msg);
    void robotiqStatesRepubCallback(const sensor_msgs::JointState::ConstPtr & msg);
};

// Update hand values from flor_grasp_controller publishing
void RobotiqRepub::robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg)
{    
    atlas_output_.rACT = msg->rACT;
    atlas_output_.rATR = msg->rATR;
    atlas_output_.rFRA = msg->rFRA;
    atlas_output_.rFRB = msg->rFRB;
    atlas_output_.rFRC = msg->rFRC;
    atlas_output_.rFRS = msg->rFRS;
    atlas_output_.rGTO = msg->rGTO;
    atlas_output_.rICF = msg->rICF;
    atlas_output_.rICS = 0;  //Individual Control of Scissor not supported
    atlas_output_.rMOD = msg->rMOD;
    atlas_output_.rPRA = msg->rPRA;
    atlas_output_.rPRB = msg->rPRB;
    atlas_output_.rPRC = msg->rPRC;
    atlas_output_.rPRS = msg->rPRS;
    atlas_output_.rSPA = msg->rSPA;
    atlas_output_.rSPB = msg->rSPB;
    atlas_output_.rSPC = msg->rSPC;
    atlas_output_.rSPS = msg->rSPS;
}

void RobotiqRepub::robotiqStatesRepubCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    //Convert joint states into RobotiqInput
    robotiq_input_.gPOA = std::max(float(0.0), std::min(float(msg->position[4]  * RAD_TO_BYTE)   ,float(255.0)));
    robotiq_input_.gPOB = std::max(float(0.0), std::min(float(msg->position[3]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_input_.gPOC = std::max(float(0.0), std::min(float(msg->position[2]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_input_.gPOS = std::max(float(0.0), std::min(float(((msg->position[0]) + SPR_ZERO) * SPR_TO_BYTE)   ,float(255.0)));
}

// Initialize hand values to 0
void RobotiqRepub::robotiqInit(std::string hand_side, std::string hand_name)
{
    robotiq_sim_output_sub   = n.subscribe("/robotiq_hands/" + hand_name + "/SModelRobotOutput", 1, &RobotiqRepub::robotiqRepubCallback, this);
    robotiq_sim_commands_pub = n.advertise<atlas_msgs::SModelRobotOutput>("/" + hand_side + "_hand/command",1);

    robotiq_sim_states_sub   = n.subscribe("/robotiq_hands/" + hand_side + "_hand/joint_states", 1, &RobotiqRepub::robotiqStatesRepubCallback, this);
    robotiq_sim_input_pub    = n.advertise<robotiq_s_model_control::SModel_robot_input>("/robotiq_hands/" + hand_name + "/SModelRobotInput",1);
}


int main(int argc, char** argv)
{
    // Create publisher
    ros::init(argc, argv, "sim_robotiq_gazebo_converter");

    std::string hand_side_;
    std::string hand_name_;

    ros::NodeHandle("~").getParam("hand_side", hand_side_);
    ros::NodeHandle("~").getParam("hand_name", hand_name_);
    ROS_INFO("hand side: %s", hand_side_.c_str());

    //Right hand class
    RobotiqRepub robotiqRepublisher;
    robotiqRepublisher.robotiqInit(hand_side_, hand_name_);

    while(ros::ok())
    {
        robotiqRepublisher.robotiq_sim_commands_pub.publish(robotiqRepublisher.atlas_output_);
        robotiqRepublisher.robotiq_sim_input_pub.publish(robotiqRepublisher.robotiq_input_);

        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
