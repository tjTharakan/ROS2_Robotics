// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <chrono>
#include <functional>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <bits/stdc++.h> 
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

#define degreeToRadians(angle) ((angle) * M_PI / 180.0)

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odometry_node")
  {
    imu_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "IMU", 10, std::bind(&OdomPublisher::marsImuUpdate, this, _1));
    rightEncoderCount_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "rightEncoderCount", 10, std::bind(&OdomPublisher::marsRightWheelVelocityCalculation, this, _1));
    leftEncoderCount_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "leftEncoderCount", 10, std::bind(&OdomPublisher::marsLeftWheelVelocityCalculation, this, _1));
    
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&OdomPublisher::odom_publisher_callback, this));
  }

private:
  rclcpp::Clock currentTimeLeftWheel,currentTimeRightWheel;
  double prevTimeLeftWheel, prevTimeRightWheel;
  double prevLeftEncoderCount = 0.0;
  double prevRightEncoderCount = 0.0;
  double prevIMU = 0.0;
  double currentIMU = 0.0;
  double x = 0.0;
  double y = 0.0;
  
  double marsLeftWheelVelocity,marsRightWheelVelocity,marsAverageVelocity,marsDeltaImuValue;
  bool imuValueCopied,leftVelocityCalculated,rightVelocityCalculated = false;

  
  rclcpp::TimerBase::SharedPtr timer_; 
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftEncoderCount_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightEncoderCount_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_subscription_;
  
  
  void odom_publisher_callback()
  {
   
    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    rclcpp::Clock currentTime;
    rclcpp::Time Time;
    double prevTime;

    nav_msgs::msg::Odometry marsOdom;	

    if(this->leftVelocityCalculated && this->rightVelocityCalculated && this->imuValueCopied)
    {
		
		this->marsAverageVelocity = ((this->marsLeftWheelVelocity + this->marsRightWheelVelocity)/2);
		RCLCPP_INFO(this->get_logger(),"Calculation average velcity is %f",this->marsAverageVelocity);
		double timeNow = double(currentTime.now().seconds());
		
		vx = this->marsAverageVelocity;
		vth = (this->marsRightWheelVelocity - this->marsLeftWheelVelocity)/0.17;

		double dt = (timeNow - prevTime);
                double dx = (vx * cos(degreeToRadians(currentIMU)) - vy * sin(degreeToRadians(currentIMU)))*dt;
		double dy = (vx * sin(degreeToRadians(currentIMU)) + vy * cos(degreeToRadians(currentIMU)))*dt;

		this->x += dx;
		this->y += dy;
		
		marsOdom.header.stamp = Time;
		marsOdom.header.frame_id = "odom";
		marsOdom.child_frame_id = "base_link";

		marsOdom.pose.pose.position.x = x;
		marsOdom.pose.pose.position.y = y;
		marsOdom.pose.pose.position.z = 0.0;

		marsOdom.twist.twist.linear.x = vx;
		marsOdom.twist.twist.linear.y = vy;
		marsOdom.twist.twist.linear.z = 0.0;
		marsOdom.twist.twist.angular.x = 0.0;
		marsOdom.twist.twist.angular.y = 0.0;
		marsOdom.twist.twist.angular.z = vth;
		
		prevTime = timeNow;	

		odom_publisher_->publish(marsOdom);	
		
		this->imuValueCopied = false;	
		this->rightVelocityCalculated = false;
 	        this->leftVelocityCalculated = false;
    }
    
  }
  
  void marsLeftWheelVelocityCalculation(const std_msgs::msg::Float64::SharedPtr marsLeftWheelEncoderCount)
   {
       if(this->leftVelocityCalculated == false)
       {

	       double deltaEncoderCount;
	       double currentEncoderCount;
	       double secondsElapsed = 0.0;
           double currentTimeLeftWheel = double(this->currentTimeLeftWheel.now().seconds());
           double prevTimeLeftWheel = this->prevTimeLeftWheel;
	       currentEncoderCount = marsLeftWheelEncoderCount->data;
	       deltaEncoderCount = currentEncoderCount - this->prevLeftEncoderCount;
	       secondsElapsed = (currentTimeLeftWheel - prevTimeLeftWheel);	
	       this->marsLeftWheelVelocity = (double)( (deltaEncoderCount * ((3.14159265 * 0.0700000)/200.0000)) / secondsElapsed );
	   
	       this->prevTimeLeftWheel = currentTimeLeftWheel;
	       this->prevLeftEncoderCount = currentEncoderCount;
	       this->leftVelocityCalculated = true;
   	    }
   }
   
   void marsRightWheelVelocityCalculation(const std_msgs::msg::Float64::SharedPtr marsRightWheelEncoderCount)
   {
       if(this->rightVelocityCalculated == false)
       {
	      
	       double deltaEncoderCount;
	       double currentEncoderCount;
	       double secondsElapsed = 0.0;
           double currentTimeRightWheel = double(this->currentTimeRightWheel.now().seconds());
	       double prevTimeRightWheel = this->prevTimeRightWheel;	
	       currentEncoderCount = marsRightWheelEncoderCount->data;
	       deltaEncoderCount = currentEncoderCount - this->prevRightEncoderCount;
	       secondsElapsed = (currentTimeRightWheel - prevTimeRightWheel);
	       this->marsRightWheelVelocity = (double)((deltaEncoderCount *((3.14159265 * 0.070000)/200.000))/ secondsElapsed );

	       this->prevTimeRightWheel = currentTimeRightWheel;
	       this->prevRightEncoderCount = currentEncoderCount;
	       this->rightVelocityCalculated = true;
               
       }
   }
    

   void marsImuUpdate(const std_msgs::msg::Float64::SharedPtr marsIMU)
   {
       if(this->imuValueCopied == false)
       {

            this->currentIMU = marsIMU->data;
            this->marsDeltaImuValue = this->currentIMU - this->prevIMU;
            this->prevIMU = this->currentIMU;
            this->imuValueCopied = true;

	   }
   }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);	
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
