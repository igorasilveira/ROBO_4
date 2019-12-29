/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/
# include "stdr_samples/mapping/mapping.h"

/**
@namespace stdr_samples
@brief The main namespace for STDR Samples
**/ 
namespace stdr_samples
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  Mapping::Mapping(int argc,char **argv)
  {
    if(argc != 3)
    {
      ROS_ERROR(
        "Usage : mapping <robot_frame_id> <laser_frame_id>");
      exit(0);
    }

    robot_name_ = std::string(argv[1]);

    laser_topic_ = std::string("/") +
      robot_name_ + std::string("/") + std::string(argv[2]);
    speeds_topic_ = std::string("/") +
      robot_name_ + std::string("/cmd_vel");
    map_topic_ = std::string("/") +
      robot_name_ + std::string("/") + "map";
      
    laserSubscriber_ = n_.subscribe(
      laser_topic_.c_str(), 
      1, 
      &Mapping::laserCallback,
      this);

    mapSubscriber_ = n_.subscribe(
      map_topic_.c_str(),
      1,
      &Mapping::mapCallback,
      this);
      
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  Mapping::~Mapping(void)
  {
    
  }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void Mapping::laserCallback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;

    getTransform();

    float linear = 0, rotational = 0;
    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      linear -= cos(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);
      rotational -= sin(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);
    }
    geometry_msgs::Twist cmd;
    
    linear /= scan_.ranges.size();
    rotational /= scan_.ranges.size();
    
    //~ ROS_ERROR("%f %f",linear,rotational);
    
    if(linear > 0.3)
    {
      linear = 0.3;
    }
    else if(linear < -0.3)
    {
      linear = -0.3;
    }

    cmd.linear.x = 0.3 + linear;
    cmd.angular.z = rotational;
    //cmd_vel_pub_.publish(cmd);
  }

  /**
  @brief Callback for the gmapping map message
  @param msg [const nav_msgs::OccupancyGrid&] The new occupancy grid message
  @return void
  **/
  void Mapping::mapCallback(const nav_msgs::OccupancyGrid& msg)
  {
    map_ = msg;

//    ROS_INFO("size: %d", map_.data.size());
//    ROS_INFO("resolution: %d", map_.info.resolution);
//    ROS_INFO("position: %d, %d, %d", map_.info.origin.position.x, map_.info.origin.position.y, map_.info.origin.position.z);
    int loop;

    for(loop = 0; loop < map_.data.size(); loop++) {
      if (map_.data.at(loop) != -1) {
        //ROS_INFO("index: %d, value: %d ", loop, map_.data.at(loop));
      }
    }
  }

  /**
  @brief Updates the saved transform of the robot
  @return void
  **/
  void Mapping::getTransform() {
    transformListener_.waitForTransform("map", robot_name_, ros::Time(), ros::Duration(1.0));
    transformListener_.lookupTransform("map", robot_name_, ros::Time(), transform_);

    double yaw, pitch, roll;
    transform_.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = transform_.getRotation();
    tf::Vector3 v = transform_.getOrigin();

    transform_info_.x = v.getX();
    transform_info_.y = v.getY();
    transform_info_.rotation = yaw;

    ROS_INFO("x: %f", transform_info_.x);
    ROS_INFO("y: %f", transform_info_.y);
    ROS_INFO("radians: %f", transform_info_.rotation);
  }
}
