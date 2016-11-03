#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include "SpeedControl.h"
#include "tools/loadRosConfig.h"

using namespace std;

// Robot dimensions in m
//       
//       
//  /-------------\  <-- MAX_X
//  |      x      |
//  |      |      |
//  |      |      |
//  | y-----      |
//  |             |
//  |    ROBOT    |
//  |             |
//  \-------------/  <-- MIN_X
//  ^             ^
//  |             |
//  MAX_Y      MIN_Y
//
float ROBOT_MIN_X = -0.30;
float ROBOT_MAX_X =  0.30;
float ROBOT_MIN_Y = -0.27;
float ROBOT_MAX_Y =  0.27;


namespace {
  Eigen::AlignedBox2f InnerDangerZone,
        OuterDangerZone;
  float InnerDangerZoneFactor,
        OuterDangerZoneFactor;

  inline Eigen::AlignedBox2f loadRect(const string& path)
  {
    pair<float, float> pX, pY;
    loadConfigValue(path + "/x_min", pX.first);
    loadConfigValue(path + "/x_max", pX.second);
    loadConfigValue(path + "/y_min", pY.first);
    loadConfigValue(path + "/y_max", pY.second);

    Eigen::Vector2f first(pX.first, pY.first), second(pX.second, pY.second);
    return Eigen::AlignedBox2f(first, second);
  }  
}

void SpeedControl::loadDimensions()
{
  InnerDangerZone = loadRect("/homer_navigation/speed_control/inner_danger_zone");
  InnerDangerZoneFactor;
  loadConfigValue("/homer_navigation/speed_control/inner_danger_zone/speed_factor", InnerDangerZoneFactor);
  OuterDangerZone = loadRect("/homer_navigation/speed_control/inner_danger_zone");
  OuterDangerZoneFactor;
  loadConfigValue("/homer_navigation/speed_control/outer_danger_zone/speed_factor", OuterDangerZoneFactor);
  if(!OuterDangerZone.contains(InnerDangerZone))
    ROS_WARN_STREAM("InnerDangerZone is not contained in OuterDangerZone");
}

float SpeedControl::getSpeedFactor(const vector<geometry_msgs::Point>& points, float minVal, float maxVal )
{
  float minFactor = 1.0;
  for (unsigned i = 0; i < points.size(); i++)
  {
    Eigen::Vector2f point(points[i].x, points[i].y);
    if(InnerDangerZone.contains(point))
    {
      minFactor = InnerDangerZoneFactor;
      break;
    }
    if(OuterDangerZone.contains(point))
      minFactor = OuterDangerZoneFactor;
  }
  minFactor = sqrt(minFactor);
  float range = maxVal - minVal;
  minFactor = minVal + range*minFactor;
  return minFactor;
}

float SpeedControl::getMaxMoveDistance(vector<geometry_msgs::Point> points)
{
  float minDistance = 4; // distance in m to nearest obstacle in front
  for (unsigned int i = 0; i < points.size(); i++)
  {
      if(points[i].y > ROBOT_MIN_Y && points[i].y < ROBOT_MAX_Y && points[i].x > ROBOT_MAX_X)
      {
        float distance = sqrt((points[i].x * points[i].x) + (points[i].y * points[i].y));
        if (distance < minDistance)
        {
          minDistance = distance;
        }
      }
  }
  float maxMoveDist = minDistance - ROBOT_MAX_X;
  if (maxMoveDist < 0) {
    maxMoveDist = 0.0;
  }
  return maxMoveDist;
}

float SpeedControl::getMaxMoveDistance(std::vector< Eigen::Vector3d >* kinectData, float minObstacleHeight, float minObstacleFromRobotDistance, float maxObstacleFromRobotDistance)
{
  // Check for obstacles in Kinect image: Look for closest point

  float minDistance = 4; // distance to nearest obstacle in front

  for(int i=0;i<kinectData->size();++i)
  {
    Eigen::Vector2d p = Eigen::Vector2d(kinectData->at(i).x(), kinectData->at(i).y());
    if(!std::isnan(p.x()))
    {
      // Filter point cloud
        if(p.x() > minObstacleFromRobotDistance && p.x() < maxObstacleFromRobotDistance && kinectData->at(i).z() > minObstacleHeight)
      {
        // Check for collisions outside of robot
        if(p.y() > ROBOT_MIN_Y && p.y() < ROBOT_MAX_Y && p.x() > ROBOT_MAX_X)
        {
          float distance = sqrt((p.x() * p.x()) + (p.y() * p.y()));
          if (distance < minDistance)
          {
            minDistance = distance;
          }
        }
      }
    }
  }

  float maxMoveDist = minDistance - ROBOT_MAX_X;
  if (maxMoveDist < 0) {
    maxMoveDist = 0.0;
  }
  return maxMoveDist;
}

float SpeedControl::getTurnSpeedFactor( float speedFactor, float turnAngle, float minVal, float maxVal )
{
  //turn faster for larger angles
  float angleDependentFactor = sqrt( fabs(turnAngle) / M_PI );
  angleDependentFactor = minVal + angleDependentFactor*(maxVal-minVal);
  return sqrt( speedFactor * angleDependentFactor );
}

float SpeedControl::getMinTurnAngle(std::vector<geometry_msgs::Point> laserData, float minAngle, float maxAngle, float minDistance, float maxDistance)
{
    float turn_factor = 1.0;
    for (unsigned int i = 0; i < laserData.size(); i++)
    {
        if(laserData[i].y > ROBOT_MIN_Y && laserData[i].y < ROBOT_MAX_Y && laserData[i].x > ROBOT_MAX_X)
        {
          float distance = sqrt((laserData[i].x * laserData[i].x) + (laserData[i].y * laserData[i].y));
          if (distance < minDistance + ROBOT_MAX_X)
          {
            turn_factor = 0.0;
          }
          else if(distance > maxDistance + ROBOT_MAX_X)
          {
              turn_factor = 1.0;
          }
          else
          {
              turn_factor = (distance - minDistance)/maxDistance;
          }
        }
    }
    float range = maxAngle - minAngle;
    return minAngle + turn_factor * range;
}

SpeedControl::SpeedControl() {
}

SpeedControl::~SpeedControl() {
}

