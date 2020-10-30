// Copyright 2020 BSH Home Appliances Corporation
//
// All rights reserved, also regarding any disposal, exploitation,
// reproduction, editing, distribution, as well as in the event of
// applications for industrial property rights.
//
// Author: Tone John Tharakan <Tone.Tharakan@bshg.com>
//
// node_clustering.cpp
//
#include <chrono>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

#define DEG2RAD(x) ((x) * M_PI / 180)

class LidarClustering : public rclcpp::Node
{
public:
  LidarClustering()
  : Node("LidarClustering")
  {
    RCLCPP_INFO(this->get_logger(),"Lidar Clustering node started");
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1000, std::bind(&LidarClustering::scanCallbackFunction, this, _1));
  }

private:

  /* Call back function triggered everytime /scan topic is subscribed.
  In this call back function, the data received from the LIDAR
  is stored into another LaserScan object and this new LaserScan object
  is used for clustering and to detect the docking station from the
  cluster */
  void scanCallbackFunction(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    scan_data_ = *scan;                                         /* Copy the scan data to local LaserScan object */
    clusterLidar();                                 /* Cluster the data avaiable in local LaserScan object*/
  }

  void clusterLidar();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  sensor_msgs::msg::LaserScan scan_data_;
  std::vector<int> clusters;
  std::vector<int> startAngleValues;
  std::vector<int> endAngleValues;
};

/* This method performs the clustering of the LIDAR data.
The method is called from the callback function and gives an output
as 'true' if atleast one cluster is detected and 'false' if no cluster
is detected */
void LidarClustering::clusterLidar()
{
  double distance = 0.0;
  double prevRange = 0.0;
  double currentRange = 0.0;
  int angleLimitHigh =0, angleLimitLow = 0;
  unsigned int clusterEnd = -1;
  std::vector<int> clusterArray;                                    /* Vector to store the clusters */

  prevRange = (scan_data_.ranges[0]);
  prevRange = prevRange * 100;                                      /* Convert range value to cm from m */

  /*Below For loop traverses through Lidar points
    starting from angle 0 and increments 1 degree per
    loop until a cluster break/end is detected */
   
  for (int i = 0; i < scan_data_.ranges.size(); i++) 
  {
    /* Fetch the current range value for index i start from 0th angle*/
    currentRange = (scan_data_.ranges[i]);
    currentRange = currentRange * 100;

    /* Calculate the distance between two consecutive Lidar points using law of cosines */
    distance = sqrt((currentRange * currentRange) + (prevRange * prevRange) - (2 * currentRange * prevRange * (cos((scan_data_.angle_increment)))));

    /* If the distance between two consecutive lidar points is less than 2 cm */
    if ((distance < 2)) 
    {
        /* Add the current index into cluster array*/
        clusterArray.push_back(i);
        
    } else 
    { /* If the distance between two consecutive lidar points is greater than 2 cm */
        angleLimitLow = i;
        break;                                                            /* Break Clustering */
    }
    prevRange = currentRange;
  }
  prevRange = scan_data_.ranges[0];
  prevRange = prevRange * 100;

  /* Below For loop traverses through Lidar points starting from angle 360
  and decrements 1 degree per loop until a cluster break/end is detected */
  for (int i = scan_data_.ranges.size()-1; i > 0; --i) 
  {
    /* Fetch the current range value for index i start from 0th angle*/
    currentRange = scan_data_.ranges[i];
    currentRange = currentRange * 100;

    /* Calculate the distance between two consecutive Lidar points using law of cosines */
    distance = sqrt((currentRange * currentRange) + (prevRange * prevRange) - (2 * currentRange * prevRange * (cos((scan_data_.angle_increment)))));

    /* If the distance between two consecutive lidar points is less than 2 cm */
    if ((distance < 2)) {
        clusterArray.push_back(i);                                          /* Add the current index into cluster array*/
    } else {                                                                /* If the distance between two consecutive lidar points is greater than 2 cm */
        clusterArray.push_back(clusterEnd);
        angleLimitHigh = i;
        break;                                                              /* Break Clustering */
    }
    prevRange = currentRange;
  }
  prevRange = scan_data_.ranges[angleLimitLow - 1];
  prevRange = prevRange * 100;

  /*The above for loops traverses from 0 degree CW to angleLimitLow
    and from 0 degree CCW till angleLimitHigh. The below for loop traverses
    from angleLimitLow to angleLimitHigh to complete traversing and
    clustering complete 360 degree LIDAR data*/

  for (int i = angleLimitLow; i < angleLimitHigh; ++i) {
    /* Fetch the current range value for index i start from 0th angle*/
    currentRange = scan_data_.ranges[i];
    currentRange = currentRange * 100;

    /* Calculate the distance between two consecutive Lidar points using law of cosines */
    distance = sqrt((currentRange * currentRange) + (prevRange * prevRange) - (2 * currentRange * prevRange * (cos((scan_data_.angle_increment)))));

    /* If the distance between two consecutive lidar points is less than 2 cm */
    if ((distance < 2)) {
        clusterArray.push_back(i);                                          /* Add the current index into cluster array*/
    } else {                                                                /* If the distance between two consecutive lidar points is greater than 2 cm */
        clusterArray.push_back(clusterEnd);                                 /* value '-1' is itroduced after the end of every cluster detected */
    }
    prevRange = currentRange;
  }

  clusters = clusterArray;
  clusterArray.clear();

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarClustering>());
  rclcpp::shutdown();
  return 0;
}
