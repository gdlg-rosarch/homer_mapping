#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include <geometry_msgs/Point.h>
#include <Eigen/Geometry>
#include <vector>

/**
 * @class SpeedControl
 * @author Malte Knauf, Stephan Wirth
 * @brief Class for computing a speed factor with respect to a given laser
 * measurement.
 */
class SpeedControl
{
public:
  /**
   * @brief Loads robot and safety zone dimensions config values
   */
  static void loadDimensions();

  /**
   * Calculates the speed factor for the robot. If a measured obstacle lies in
   * the "danger zone"
   * that is defined in SpeedControl.cpp, the speed factor will be below maxVal.
   * The nearer the obstacle,
   * the smaller the speed factor.
   * @param laserData Laser measurement
   * @param minVal,maxVal range of return values
   * @return Speed factor, value between minVal and maxVal. The higher the speed
   * factor, the safer is it to drive fast.
   */
  static float getSpeedFactor(const std::vector<geometry_msgs::Point>& points,
                              float minVal = 0.2, float maxVal = 1.0);

  /**
   * Calculates the maximum distance the robot can move without touching an
   * obstacle.
   * @param laserPoints Current laser measurement transformed to (valid) points
   * in map frame
   * @param laserConf The configuration of the LRF that took the measurement
   * @return maximum distance (m) the robot can move based on the given
   * laserscan.
   */
  static float getMaxMoveDistance(std::vector<geometry_msgs::Point> laserData);

  static float getMaxMoveDistance(std::vector<Eigen::Vector3d>* kinectData,
                                  float minObstacleHeight,
                                  float minObstacleFromRobotDistance,
                                  float maxObstacleFromRobotDistance);

  /// @return if the angle is larger, the turn speed factor will be higher
  static float getTurnSpeedFactor(float speedFactor, float turnAngle,
                                  float minVal, float maxVal);

  /**
   * Calculates the minimum angle between the robot's orientation and the next
   * waypoint which is necessary
   * to trigger a rotation instead of a straight line
   * @brief getMinTurnAngle
   * @param laserData
   * @param minAngle
   * @param maxAngle
   * @return
   */
  static float getMinTurnAngle(std::vector<geometry_msgs::Point> laserData,
                               float minAngle, float maxAngle,
                               float minDistance, float maxDistance);

private:
  /**
   * Constructor is empty and private because this class will never be
   * instanciated.
   */
  SpeedControl();

  /**
   * Destructor is empty.
   */
  ~SpeedControl();
};
#endif
