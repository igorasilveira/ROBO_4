#ifndef STDR_MAPPING_SAMPLE
#define STDR_MAPPING_SAMPLE

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <stdlib.h>

#include <tf/transform_listener.h>

#include <ros/package.h>
#include "ros/ros.h"

#include <stdr_msgs/RobotIndexedVectorMsg.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

/**
@namespace stdr_samples
@brief The main namespace for STDR Samples
**/ 
namespace stdr_samples
{

  struct transform_info {
    float x;
    float y;
    float rotation;
  };

  struct Entrance {
    float left_wall_x_;
    float left_wall_y_;
    float right_wall_x_;
    float right_wall_y_;
  };

  struct Room {
    Entrance entrance_;
    float top_left_corner_x_;
    float top_left_corner_y_;
    float top_right_corner_x_;
    float top_right_corner_y_;
    float bottom_left_corner_x_;
    float bottom_left_corner_y_;
    float bottom_right_corner_x_;
    float bottom_right_corner_y_;
    float area;
  };

  enum RoomPosition {
      ABOVE, BELOW
  };

  enum MappingStates {
    FIND_ROOM, GO_TO_ROOM, MAP_ROOM, EXIT_ROOM
  };

  enum FindRoomStates {
    FIND_ENTRANCES,
    MOVE_AROUND
  };

  enum RoomPositioningStates {
    POSITIONING_ROTATION,
    ALIGNMENT_MOVEMENT,
    ENTRANCE_ROTATION,
    ROOM_ENTER
  };

  enum MapRoomStates {
    FIND_CORNER
  };
  /**
  @class Mapping
  @brief Performs mapping to a single robot
  **/ 
  class Mapping
  {
    private:

      float MAX_ANGULAR_SPEED_;
      float MAX_LINEAR_SPEED_;

      bool has_set_initial_y_;

      bool found_all_rooms_;
      bool has_set_biggest_;

      tf::TransformListener transformListener_;

      tf::StampedTransform transform_;

      //!< The gmapping map resolution
      float initial_y_;

      //!< The gmapping map resolution
      double map_resolution_;

      //!< The already mapped rooms
      std::vector<Room> mapped_rooms_;

      //!< The already known entrances
      std::vector<Entrance> mapped_entrances_;

      //!< The current assigned entrance
      Entrance current_entrance_;

      //!< The current mapping room
      Room current_room_;

      //!< The current room position
      RoomPosition current_room_position;

      //!< The current major state
      MappingStates current_state_;

      //!< The current FindRoomStates state
      FindRoomStates current_find_room_state;

      //!< The current GoToRoomStates state
      RoomPositioningStates current_room_positioning_state;

      //!< The current MapRoomStates state
      MapRoomStates current_map_room_state;

      //!< The robot frame id
      std::string robot_name_;

      //!< The robot transform info
      transform_info transform_info_;

      //!< The ros laser scan msg
      sensor_msgs::LaserScan scan_;

      //!< The gmmaping map msg
      nav_msgs::OccupancyGrid map_;

      //!< Subscriber for the ros laser msg
      ros::Subscriber laserSubscriber_;

      //!< Subscriber for the gmapping map msg
      ros::Subscriber mapSubscriber_;

      //!< The ROS node handle
      ros::NodeHandle n_;
      
      //!< The laser topic
      std::string laser_topic_;
      
      //!< The map topic
      std::string map_topic_;

      //!< The speeds topic
      std::string speeds_topic_;
      
      //!< The twist publisher
      ros::Publisher cmd_vel_pub_;
      
      float old_x_;

    public:
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      Mapping(int argc,char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~Mapping(void);
      
      /**
      @brief Callback for the ros laser message
      @param msg [const sensor_msgs::LaserScan&] The new laser scan message
      @return void
      **/
      void laserCallback(const sensor_msgs::LaserScan& msg);

      /**
      @brief Callback for the gmapping map message
      @param msg [const nav_msgs::OccupancyGrid&] The new occupancy grid message
      @return void
      **/
      void mapCallback(const nav_msgs::OccupancyGrid& msg);

      /**
      @brief Gets the index of the occupancy grid data[]
      @param x [const int] the x coordinate
      @param y [const int] the y coordinate
      @return int the index
      **/
      int getMapDataIndex(const float x, const float y);

      /**
      @brief Updates the saved transform of the robot
      @return void
      **/
      void getTransform();

      void findRoom();

      void goToRoom();

      void mapRoom();

      void exitRoom();

      /**
      @brief Find y coordinate of vertical wall
      @param const int direction to find, 1 is upwards
      @return int the y coordinate
      **/
      int findVerticalWall(const int direction);

      /**
      @brief Finds the entrances along a wall
      @param const int y coordinate of the wall
      **/
      void findWallEntrances(const int y);

      /**
      @brief Finds the entrances
      **/
      void findEntrances();

      /**
      @brief Selects one entrance that has not been used yet
      @return True if found valid entrance
      **/
      bool selectEntrance();

      /**
      @brief Rotates towards a given degree
      **/
      bool rotateToDegree(float degree);

      /**
      @brief Rotates towards entrance middle point
      **/
      void rotateToEntrancePositioning();

      /**
      @brief Moves towards entrance middle point
      **/
      void moveToEntrancePositioning();

      /**
      @brief Rotates towards entrance middle point when bellow it
      @param Whether it is an exit
      **/
      void rotateToEntrance(bool isExit);

      /**
      @brief Enters a room until at 2 meter of wall
      **/
      void enterRoom();

      /**
      @brief Exits a room until at initial y position
      **/
      void exitToHall();

      /**
      @brief Exits a room until at initial y position
      @return True if all 4 corners were found
      **/
      bool findCorners();

      /**
      @brief Moves around to initialize
      **/
      void moveAround();

      /**
      @brief Sets the current room to be the largest
      **/
      void setLargestRoom();

      bool moveToX(float degree);
  };
}

#endif
