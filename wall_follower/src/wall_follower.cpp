// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>
#include <unistd.h>

using namespace std::chrono_literals;
// double[360] arrayAll;
WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	// for (int i = 0;)
	for (int i = CENTER; i < RIGHTBACK; i++) scan_data_[i] = 0.0;
	for (int i = 0; i < 30; i++) left_scan_data[i] = 0.0;
	for (int i = 0; i < 44; i++) front_scan_data[i] = 0.0;
	// for (int i = 0; i < 365; i++) arrayAll[i] = 0.0;
	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[7] = {0, 90, 270, 125, 55, 0, 0};
	for (int i = 0; i < 30; i++) left_scan_data[i] = std::isinf(msg->ranges.at(75+i)) ? msg->range_max : msg->ranges.at(75 + i);
	for (int i = 0; i < 22; i++) front_scan_data[i] = std::isinf(msg->ranges.at(0+i)) ? msg->range_max : msg->ranges.at(0 + i);
	for (int i = 22; i < 44; i++) front_scan_data[i] = std::isinf(msg->ranges.at(316+i)) ? msg->range_max : msg->ranges.at(316 + i);
	for (int num = 0; num < 7; num++)
	{	
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = GET_TB3_DIRECTION;

	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:
			// std::this_thread::sleep_for(std::chrono::milliseconds(100));
			prev_robot_pose_ = robot_pose_;
			// Logic checks which direction robot should be moving
			if (!wall_on_diagonal() && !wall_on_front()) {
				turtlebot3_state_num = TB3_LEFT_TURN;
				std::cout << "Left turn\n";
			}
			else if (wall_on_front() || scan_data_[LEFTFRONT] < 0.5) {
				turtlebot3_state_num = TB3_RIGHT_TURN;
				std::cout << "Right turn\n";
			}
			else {
				turtlebot3_state_num = TB3_DRIVE_FORWARD;
				std::cout << "Forwards\n";
			}
			break;

		case TB3_DRIVE_FORWARD:
			stabilise();
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_RIGHT_TURN:
			// Turn on spot
			update_cmd_vel(0.0, -1 * (ANGULAR_VELOCITY));
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_LEFT_TURN:
			// Arc turn
			update_cmd_vel(LINEAR_VELOCITY * 0.9, ANGULAR_VELOCITY * 0.5);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
	}
}

	/*******************************************************************************
	** Helper Functions
	*******************************************************************************/

bool WallFollower::wall_on_left() {
	for (int i = 0; i < 30; i++) {
		if (left_scan_data[i] < SIDE_DIST) return true;
	}
	return false;
}
bool WallFollower::wall_on_front() {
	int counter = 0;
	for (int i = 0; i < 44; i++) {
		if (front_scan_data[i] < FRONT_DIST) counter++;
		if (counter >= 25) return true;
	}
	return false;
}

bool WallFollower::wall_on_diagonal() {
	return (scan_data_[LEFTFRONT] < DIAGONAL_DIST);
}

bool WallFollower::left_wall_flush() {
	return (fabs(left_scan_data[0] - left_scan_data[29]) < 0.05);
}

void WallFollower::stabilise() {
	double towards_wall = left_scan_data[0] - left_scan_data[29];
	if (towards_wall < -0.02 || left_scan_data[15] < 0.1) {
		std::cout << "stabilise away\n";
		//update_cmd_vel(LINEAR_VELOCITY, -0.15 * ANGULAR_VELOCITY);
	} else if (towards_wall > 0.02) {
		std::cout << "stabilse towards\n";
		update_cmd_vel(LINEAR_VELOCITY, 0.25 * ANGULAR_VELOCITY);
	} else {
		update_cmd_vel(LINEAR_VELOCITY, 0);
	}
	return;
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
