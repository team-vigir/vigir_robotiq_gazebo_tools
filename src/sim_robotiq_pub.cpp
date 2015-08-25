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
#include "robotiq_s_model_control/SModel_robot_input.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define ROBOTIQ_NUM_JOINTS 11

class RobotiqRepub
{
    public:
    ros::NodeHandle n;
    ros::Publisher  robotiq_sim_output_pub;
    ros::Subscriber grasp_controller_sub;

    robotiq_s_model_control::SModel_robot_input hand_joints_;

    void robotiqInit(std::string hand);
    void robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr & msg);
};

// Update hand values from flor_grasp_controller publishing
void RobotiqRepub::robotiqRepubCallback(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg)
{

    //Copying the requested position
    this->hand_joints_.gPRA = msg->rPRA;
    this->hand_joints_.gPRB = msg->rPRB;
    this->hand_joints_.gPRC = msg->rPRC;
    this->hand_joints_.gPRS = msg->rPRS;

    //Faking unperfect controller with +-2% error
    double error;

    /* initialize random seed: */
    srand (time(NULL));

    /* generate error between 1 and 10: */
    //error = rand() % 10 -5;
    error = 0;

    this->hand_joints_.gPOA = std::max(float(0.0), std::min(float(msg->rPRA + error),float(255.0)));
    this->hand_joints_.gPOB = std::max(float(0.0), std::min(float(msg->rPRB + error),float(255.0)));
    this->hand_joints_.gPOC = std::max(float(0.0), std::min(float(msg->rPRC + error),float(255.0)));
    this->hand_joints_.gPOS = std::max(float(0.0), std::min(float(msg->rPRS + error),float(255.0)));

    //Faking currents depending on force (20%), just to have some fake feedback
    this->hand_joints_.gCUA = msg->rFRA * 0.2;
    this->hand_joints_.gCUB = msg->rFRB * 0.2;
    this->hand_joints_.gCUC = msg->rFRC * 0.2;
    this->hand_joints_.gCUS = msg->rFRS * 0.2;
}

// Initialize hand values to 0
void RobotiqRepub::robotiqInit(std::string hand)
{
    this->hand_joints_.gPOA = 0;
    this->hand_joints_.gPOB = 0;
    this->hand_joints_.gPOB = 0;
    this->hand_joints_.gPOS = 137;
    grasp_controller_sub   = n.subscribe("/robotiq_hands/" + hand + "/SModelRobotOutput", 1, &RobotiqRepub::robotiqRepubCallback, this);
    robotiq_sim_output_pub = n.advertise<robotiq_s_model_control::SModel_robot_input>("/robotiq_hands/" + hand + "/SModelRobotInput",1);
}


int main(int argc, char** argv)
{
    // Create publisher
    ros::init(argc, argv, "sim_robotiq_pub");

    std::string hand_side_;
    std::string hand_name_;

    ros::NodeHandle("~").getParam("hand_side", hand_side_);
    ros::NodeHandle("~").getParam("hand_name", hand_name_);
    ROS_INFO("hand side: %s", hand_side_.c_str());


    //Right hand class
    RobotiqRepub robotiqRepublisher;
    robotiqRepublisher.robotiqInit(hand_name_);

    while(ros::ok())
    {
        robotiqRepublisher.robotiq_sim_output_pub.publish(robotiqRepublisher.hand_joints_);

        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
