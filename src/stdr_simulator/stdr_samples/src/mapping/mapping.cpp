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

    initial_y_ = 0;

    has_set_initial_y_= false;

    found_all_rooms_ = false;
    has_set_biggest_ = false;

    old_x_ = -1;

    MAX_ANGULAR_SPEED_ = .8;
    MAX_LINEAR_SPEED_ = 0.3;

    //!< The current major state
    current_state_ = FIND_ROOM;

    //!< The current FindRoomStates state
    current_find_room_state = MOVE_AROUND;

    //!< The current GoToRoomStates state
    current_room_positioning_state = POSITIONING_ROTATION;

    //!< The current MapRoomStates state
    current_map_room_state = FIND_CORNER;

    current_room_position = BELOW;

    map_resolution_ = 0.25;

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

    if (!has_set_initial_y_) {
      has_set_initial_y_ = true;
      initial_y_ = transform_info_.y;
    }

    if (old_x_ == -1) {
      old_x_ = transform_info_.x;
    }

    if (found_all_rooms_) {
      if (!has_set_biggest_) {
        has_set_biggest_ = true;
        setLargestRoom();
      }
      goToRoom();
    } else {

      switch (current_state_) {
        case FIND_ROOM:
          findRoom();
          break;
        case GO_TO_ROOM:
          goToRoom();
          break;
        case MAP_ROOM:
          mapRoom();
          break;
        case EXIT_ROOM:
          exitRoom();
          break;
        default:
          break;
      }
    }

  }

  /**
  @brief Sets the current room to be the largest
  **/
  void Mapping::setLargestRoom() {
    float maxArea = 0;
    int maxRoom = 0;
    for (int j = 0; j < mapped_rooms_.size(); ++j) {
      if (mapped_rooms_.at(j).area > maxArea) {
        maxArea = mapped_rooms_.at(j).area;
        maxRoom = j;
      }
    }

    current_room_ = mapped_rooms_.at(maxRoom);
    current_entrance_ = current_room_.entrance_;
  }

  /**
  @brief Callback for the gmapping map message
  @param msg [const nav_msgs::OccupancyGrid&] The new occupancy grid message
  @return void
  **/
  void Mapping::mapCallback(const nav_msgs::OccupancyGrid& msg)
  {
    map_ = msg;

    getTransform();

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
  }

  /**
  @brief Gets the index of the occupancy grid data[]
  @param x [const int] the x coordinate
  @param y [const int] the y coordinate
  @return int the index
  **/
  int Mapping::getMapDataIndex(const float x, const float y) {
    return x + y * map_.info.width;
  }

  /**
  @brief Find y coordinate of vertical wall
  @param const int direction to find, 1 is upwards
  @return int the y coordinate
  **/
  int Mapping::findVerticalWall(const int direction) {
    bool found = false;
    int counter = 1, xIncrement = 0, yIncrement;

    float searchX = (int)transform_info_.x / map_resolution_,
        searchY = (int)transform_info_.y / map_resolution_;

    while (!found) {
      // is it further then 2 meters? update x
      if (abs(yIncrement * map_resolution_) > 2) {
        counter = 1;
        xIncrement++;
      }

      yIncrement = counter * direction;

      int mapValue = map_.data.at(getMapDataIndex(searchX + xIncrement, searchY + yIncrement));

      found = mapValue == 100;
      counter++;
    }

    return (int)transform_info_.y / map_resolution_ + yIncrement;
  }

  /**
  @brief Finds the entrances
  **/
  void Mapping::findEntrances() {
    // wall top

    Mapping::findWallEntrances(Mapping::findVerticalWall(1));


    // wall bottom
    Mapping::findWallEntrances(Mapping::findVerticalWall(-1));
  }

  /**
  @brief Finds the entrances along a wall
  @param const int y coordinate of the wall
  **/
  void Mapping::findWallEntrances(const int y) {
    int xLeft = 0;
    while (xLeft < map_.info.width) {
      int mapValue = map_.data.at(getMapDataIndex(xLeft, y));

      if (mapValue == 100) {
        int entranceMapValue = mapValue;
        int entranceXRight = -1;
        bool startedEntrance = false;

        // if the block after is 0, is finding entrance
        if (map_.data.at(getMapDataIndex(xLeft + 1, y)) == 0) {
          startedEntrance = true;
          xLeft++;

          Entrance entrance = Entrance();
          entrance.left_wall_x_ = xLeft;
          entrance.left_wall_y_ = y;
          entrance.right_wall_y_ = y;
          bool forceBreak = false;

          while (entranceXRight == -1 && !forceBreak) {
            if (xLeft < map_.info.width) {
              entranceMapValue = map_.data.at(getMapDataIndex(xLeft, y));

              if (entranceMapValue == 100 &&
                  startedEntrance &&
                  map_.data.at(getMapDataIndex(entrance.left_wall_x_ + 1, y - 1) != 100) &&
                  abs(xLeft - 1 - entrance.left_wall_x_) <= 2 / map_resolution_) {
                entranceXRight = xLeft - 1;
                entrance.right_wall_x_ = entranceXRight;
              }

              xLeft++;
            } else {
              forceBreak = true;
            }
          }

          if (entranceXRight != -1) {
            mapped_entrances_.push_back(entrance);

            ROS_INFO("Found entrance at (%f, %f) to (%f, %f)",
                     entrance.left_wall_x_,
                     entrance.left_wall_y_,
                     entrance.right_wall_x_,
                     entrance.right_wall_y_);
          }
        }

      }
      xLeft++;
    }
  }

  /**
  @brief Moves around to initialize
  **/
  void Mapping::moveAround() {
    geometry_msgs::Twist cmd;

    if (rotateToDegree(0)) {

      while (!moveToX(old_x_ + 1.0)) {

      }

      cmd.linear.x = 0;
      cmd_vel_pub_.publish(cmd);
      current_find_room_state = FIND_ENTRANCES;
    }
  }

  void Mapping::findRoom() {
    if (current_find_room_state == FIND_ENTRANCES) {
      findEntrances();

      if (mapped_entrances_.size() > 0) {
        if (selectEntrance()) {
          current_state_ = GO_TO_ROOM;
          current_room_positioning_state = POSITIONING_ROTATION;
          return;
        }
      }

      current_find_room_state = MOVE_AROUND;
    }

    if (current_find_room_state == MOVE_AROUND) {
      old_x_ = transform_info_.x;
      moveAround();
    }
  }

  /**
  @brief Rotates towards a given degree
  **/
  bool Mapping::rotateToDegree(float degree) {
    geometry_msgs::Twist cmd;

    cmd.angular.z = MAX_ANGULAR_SPEED_ * (degree - transform_info_.rotation);
    cmd_vel_pub_.publish(cmd);

    if (fabs(degree - transform_info_.rotation) <= 0.01) {
      return true;
    }

    return false;
  }

  /**
  @brief Rotates towards a given degree
  **/
  bool Mapping::moveToX(float x) {
    geometry_msgs::Twist cmd;
    getTransform();

    cmd.linear.x = MAX_LINEAR_SPEED_ * fabs(x - transform_info_.x);
    cmd_vel_pub_.publish(cmd);

    if (fabs(x - transform_info_.x) <= 0.1) {
      return true;
    }

    int searchX = (int)transform_info_.x / map_resolution_;
    int searchY = (int)transform_info_.y / map_resolution_;
    int counter = 0;

    for (counter = 0; abs(counter) < (2.0 / map_resolution_);) {
      if (map_.data.at(getMapDataIndex(searchX + counter, searchY)) > 40) {
        ROS_INFO("searchX %d", searchX);
        ROS_INFO("searchY %d", searchY);
        ROS_INFO("counter %d", counter);
        ROS_INFO("value %d", map_.data.at(getMapDataIndex(searchX + counter, searchY)));
        if (!selectEntrance()) {
          ROS_INFO("FOUND ALL ROOMS");
          found_all_rooms_ = true;
        }

        return true;
      }

      counter += 1;
    }

    return false;
  }

  /**
  @brief Rotates towards entrance middle point
  **/
  void Mapping::rotateToEntrancePositioning() {
    float middlePointX;

    middlePointX = (current_entrance_.right_wall_x_ -
        ((current_entrance_.right_wall_x_ - current_entrance_.left_wall_x_) / 2)) * map_resolution_;

    float objectiveRotation = 0;

    if (middlePointX < transform_info_.x - 0.2) {
      objectiveRotation = M_PI;
    } else if (middlePointX > transform_info_.x + 0.2) {
      objectiveRotation = 0;
    } else {
      current_room_positioning_state = ENTRANCE_ROTATION;
      return;
    }

    if (rotateToDegree(objectiveRotation)) {
      current_room_positioning_state = ALIGNMENT_MOVEMENT;
    }
  }

  /**
  @brief Rotates towards entrance middle point when below it
  **/
  void Mapping::rotateToEntrance(bool isExit) {
    float middlePointX;

    float objectiveRotation = 0;

    if (isExit) {
      objectiveRotation = current_room_position == ABOVE ? -M_PI_2 : M_PI_2;
    } else {
      if (current_entrance_.left_wall_y_* map_resolution_ > transform_info_.y) {
        objectiveRotation = M_PI_2;
        current_room_position = ABOVE;
      }

      if (current_entrance_.left_wall_y_ * map_resolution_ <= transform_info_.y) {
        objectiveRotation = -M_PI_2;
        current_room_position = BELOW;
      }
    }

    if (rotateToDegree(objectiveRotation)) {
      current_room_positioning_state = ROOM_ENTER;
    }
  }

  /**
  @brief Moves towards entrance middle point
  **/
  void Mapping::moveToEntrancePositioning() {
    geometry_msgs::Twist cmd;

    float middlePointX;

    middlePointX = (current_entrance_.right_wall_x_ -
        ((current_entrance_.right_wall_x_ - current_entrance_.left_wall_x_) / 2)) * map_resolution_;

    float minPosition = middlePointX - 0.01;
    float maxPosition = middlePointX + 0.01;

    if (fabs(transform_info_.x - minPosition) <= (maxPosition - minPosition)) {
      cmd.linear.x = 0;
      cmd_vel_pub_.publish(cmd);
      current_room_positioning_state = ENTRANCE_ROTATION;
    }
    else {
      cmd.linear.x = MAX_LINEAR_SPEED_;
      cmd_vel_pub_.publish(cmd);
    }
  }

  /**
  @brief Enters a room until at 1 meter of wall
  **/
  void Mapping::enterRoom() {
    geometry_msgs::Twist cmd;

    int mapValue = -1, yIncrement, counter = 0;
    float distance = 1.5;
    bool hasFound = false;

    int direction = current_room_position == ABOVE ? 1 : -1;

    int searchX = (int)transform_info_.x / map_resolution_;
    int searchY = (int)transform_info_.y / map_resolution_;

    if (fabs(searchY * map_resolution_ - initial_y_) > 2) {
      for (counter = 0; abs(counter) < (distance / map_resolution_);) {
        if (map_.data.at(getMapDataIndex(searchX, searchY + counter)) == 100) {
          hasFound = true;
          break;
        }

        counter += direction;
      }
    }

    if (!hasFound) {
      cmd.linear.x = MAX_LINEAR_SPEED_ / 2;
      cmd_vel_pub_.publish(cmd);
    } else {
      cmd.linear.x = 0;
      cmd_vel_pub_.publish(cmd);

      if (!found_all_rooms_) {
        current_room_positioning_state = POSITIONING_ROTATION;
        current_state_ = MAP_ROOM;
      } else {
        ROS_INFO("SUCCESS");
        exit(0);
      }
    }

  }

  void Mapping::goToRoom() {
    switch (current_room_positioning_state) {
      case POSITIONING_ROTATION:
        rotateToEntrancePositioning();
        break;
      case ALIGNMENT_MOVEMENT:
        moveToEntrancePositioning();
        break;
      case ENTRANCE_ROTATION:
        rotateToEntrance(false);
        break;
      case ROOM_ENTER:
        enterRoom();
        break;
      default:
        break;
    }
  }

  /**
  @brief Exits a room until at initial y position
  @return True if all 4 corners were found
  **/
  bool Mapping::findCorners() {
    int topFreeRow = findVerticalWall(1) - 1;

    float searchX = (int)transform_info_.x / map_resolution_;
    int xIncrement = 0;

    bool foundTopLeft = false, foundTopRight = false,
        foundBottomLeft = false, foundBottomRight = false;

    int mapValue = 0;

    // find top left
    while (mapValue != -1 && !foundTopLeft) {
      xIncrement--;

      mapValue = map_.data.at(getMapDataIndex(searchX + xIncrement, topFreeRow));

      if (mapValue == 100) {
        foundTopLeft = true;
        current_room_.top_left_corner_x_ = searchX + xIncrement;
        current_room_.top_left_corner_y_ = topFreeRow;
      }
    }

    xIncrement = 0; mapValue = 0;

    // find top right
    while (mapValue != -1 && !foundTopRight) {
      xIncrement++;

      mapValue = map_.data.at(getMapDataIndex(searchX + xIncrement, topFreeRow));

      if (mapValue == 100) {
        foundTopRight = true;
        current_room_.top_right_corner_x_ = searchX + xIncrement;
        current_room_.top_right_corner_y_ = topFreeRow;
      }
    }

    xIncrement = 0; mapValue = 0;

    int bottomFreeRow = findVerticalWall(-1) + 1;

    // find bottom left
    while (mapValue != -1 && !foundBottomLeft) {
      xIncrement--;

      mapValue = map_.data.at(getMapDataIndex(searchX + xIncrement, bottomFreeRow));

      if (mapValue == 100) {
        foundBottomLeft = true;
        current_room_.bottom_left_corner_x_ = searchX + xIncrement;
        current_room_.bottom_left_corner_y_ = bottomFreeRow;
      }
    }
    xIncrement = 0; mapValue = 0;

    // find bottom right
    while (mapValue != -1 && !foundBottomRight) {
      xIncrement++;

      mapValue = map_.data.at(getMapDataIndex(searchX + xIncrement, bottomFreeRow));

      if (mapValue == 100) {
        foundBottomRight = true;
        current_room_.bottom_right_corner_x_ = searchX + xIncrement;
        current_room_.bottom_right_corner_y_ = bottomFreeRow;
      }
    }

    bool validForCalculation = (foundTopLeft && foundBottomRight) || (foundTopRight&& foundBottomLeft);

    // calculate room area
    if (validForCalculation) {
      if (foundTopLeft && foundBottomRight) {
        current_room_.area =
            fabs((float)current_room_.top_left_corner_x_ * map_resolution_ - current_room_.bottom_right_corner_x_ * map_resolution_) *
            fabs(current_room_.top_left_corner_y_ * map_resolution_ - current_room_.bottom_right_corner_y_ * map_resolution_);
      } else if (foundTopRight&& foundBottomLeft) {
        current_room_.area =
            fabs((float)current_room_.top_right_corner_x_ - current_room_.bottom_left_corner_x_) *
            fabs(current_room_.top_right_corner_y_ - current_room_.bottom_left_corner_y_);
      }
    }

    return validForCalculation;
  }

  void Mapping::mapRoom() {
    if (findCorners()) {
      mapped_rooms_.push_back(current_room_);
      current_room_ = Room();
      current_state_ = EXIT_ROOM;
    }
  }


  /**
  @brief Exits a room until at initial y position
  **/
  void Mapping::exitToHall() {
    geometry_msgs::Twist cmd;

    if (fabs(initial_y_ - transform_info_.y) > 0.1) {
      cmd.linear.x = MAX_LINEAR_SPEED_* 2;
      cmd_vel_pub_.publish(cmd);
    } else {
      cmd.linear.x = 0;
      cmd_vel_pub_.publish(cmd);

      current_room_positioning_state = POSITIONING_ROTATION;
      current_state_ = FIND_ROOM;
    }
  }

  void Mapping::exitRoom() {
    switch (current_room_positioning_state) {
      case POSITIONING_ROTATION:
        rotateToEntrancePositioning();
        break;
      case ALIGNMENT_MOVEMENT:
        moveToEntrancePositioning();
        break;
      case ENTRANCE_ROTATION:
        rotateToEntrance(true);
        break;
      case ROOM_ENTER:
        exitToHall();
        break;
      default:
        break;
    }
  }

  /**
  @brief Selects one entrance that has not been used yet
  **/
  bool Mapping::selectEntrance() {
    if (mapped_rooms_.size() > 0) {
      for (int i = 0; i < mapped_entrances_.size(); ++i) {
        bool validEntrance = true;

        for (int j = 0; j < mapped_rooms_.size(); ++j) {
          if (
              mapped_rooms_.at(j).entrance_.left_wall_x_ == mapped_entrances_.at(i).left_wall_x_ &&
              mapped_rooms_.at(j).entrance_.right_wall_x_ == mapped_entrances_.at(i).right_wall_x_
              ) {
            validEntrance = false;
          }
        }

        if (validEntrance) {
          current_entrance_ = mapped_entrances_.at(i);
          current_room_ = Room();
          current_room_.entrance_ = current_entrance_;
          return true;
        }
      }

      return false;
    } else {
      current_entrance_ = mapped_entrances_.at(0);
      current_room_ = Room();
      current_room_.entrance_ = current_entrance_;
      return true;
    }

    return false;
  }
}
