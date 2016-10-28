/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "sensor_msgs/JointState.h"

int main(int argc, char** argv)
{
	srand(1);
	ros::init(argc, argv, "kinematics_solver");
	ros::NodeHandle nh("~");

	std::string chain_start, chain_end, urdf_param;
	double timeout;

	nh.param("chain_start", chain_start, std::string(""));
	nh.param("chain_end", chain_end, std::string(""));
  
	if (chain_start=="" || chain_end=="") {
		ROS_FATAL("Missing chain info in launch file");
		exit (-1);
	}

	nh.param("timeout", timeout, 0.005);
	nh.param("urdf_param", urdf_param, std::string("/robot_description"));

	double eps = 1e-5;

	// This constructor parses the URDF loaded in rosparm urdf_param into the
	// needed KDL structures.  We then pull these out to compare against the KDL
	// IK solver.
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);
  
	if (!valid) {
		ROS_ERROR("There was no valid KDL chain found");
		return -1;
	}

	valid = tracik_solver.getKDLLimits(ll,ul);

	if (!valid) {
		ROS_ERROR("There were no valid KDL joint limits found");
		return -1;
	}

	ll(0) = -3.14;
	ll(1) = -3.14;
	ll(2) = -3.14;
	ul(0) = 0;
	ul(1) = 0;
	ul(2) = 0;
	ul(3) = 0;

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	ROS_INFO ("Using %d joints",chain.getNrOfJoints());

	// Create Nominal chain configuration midway between all joint limits
  	KDL::JntArray nominal(chain.getNrOfJoints());

  	for (uint j=0; j<nominal.data.size(); j++) {
    		nominal(j) = (ll(j)+ul(j))/2.0;
	}

	KDL::JntArray result;
	KDL::Frame end_effector_pose;
	int rc;

	end_effector_pose = KDL::Frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(0.0, 0.3, 0.2));
	rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
	/*while (!((result(0) < 0) && (result(1) < 0) && (result(2) < 0) && (result(3) < 0)))
	{
		ROS_INFO_STREAM(result(0));
		ROS_INFO_STREAM(result(1));
		ROS_INFO_STREAM(result(2));
		ROS_INFO_STREAM(result(3));
		ROS_INFO_STREAM(result(4));
		ROS_INFO_STREAM(result(5));
		rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
	}*/

	ROS_INFO_STREAM(result(0));
	ROS_INFO_STREAM(result(1));
	ROS_INFO_STREAM(result(2));
	ROS_INFO_STREAM(result(3));
	ROS_INFO_STREAM(result(4));
	ROS_INFO_STREAM(result(5));

	ros::Publisher kinematics_pub = nh.advertise<sensor_msgs::JointState>("kinematics_talker", 1000);

	while (ros::ok())
	{
		sensor_msgs::JointState msg;
		msg.position.push_back(result(0));
		msg.position.push_back(result(1));
		msg.position.push_back(result(2));
		msg.position.push_back(result(3));
		msg.position.push_back(result(4));
		msg.position.push_back(result(5));

		kinematics_pub.publish(msg);
	}

	// Useful when you make a script that loops over multiple launch files that test different robot chains
	// std::vector<char *> commandVector;
	// commandVector.push_back((char*)"killall");
	// commandVector.push_back((char*)"-9");
	// commandVector.push_back((char*)"roslaunch");
	// commandVector.push_back(NULL);  

	// char **command = &commandVector[0];
	// execvp(command[0],command);

	return 0;
}
