#ifndef MAPMANAGER_H
#define MAPMANAGER_H

#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <homer_map_manager/Managers/MaskingManager.h>

#include <geometry_msgs/Pose.h>
#include <homer_mapnav_msgs/MapLayers.h>
#include <homer_mapnav_msgs/ModifyMap.h>
#include <homer_nav_libs/tools.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include "ros/ros.h"

/** @class MapManager
  * @author Malte Knauf, David Gossow (RX), Susanne Maur
  * @brief This class holds all current map layers, updates them and publishes
 * them in one merged map.
  */
class MapManager
{
public:
  /**
   * @brief Constructor
   * @param nh node handle
   */
  MapManager(ros::NodeHandle* nh);

  /**
   * @brief getMapLayer search for map layer of given type and return it
   * @param type type of map layer to search for
   * @return map layer
   */
  nav_msgs::OccupancyGrid::ConstPtr getMapLayer(int type);

  /**
   * @brief updateMapLayer replaces map layer of given type
   * @param type type of map layer
   * @param layer new map layer
   */
  void updateMapLayer(int type, nav_msgs::OccupancyGrid::ConstPtr layer);

  /**
   * @brief toggleMapVisibility toggles visibility of each map layer
   * @param type type of map layer to toggle
   * @param state visible or not
   */
  void toggleMapVisibility(int type, bool state);

  /**
   * @brief clearMapLayers Clear all map layers
   */
  void clearMapLayers();

  /** merges all map layers and publishes the merged map */
  void sendMergedMap();

  void updateMaskingManager(MaskingManager* maskingManager)
  {
    m_MaskingManager = maskingManager;
  }

  /** destructor */
  virtual ~MapManager();

private:
  /**
   * The map data of each available map layer ist stored in this map
   */
  std::map<int, nav_msgs::OccupancyGrid::ConstPtr> m_MapLayers;
  std::map<int, sensor_msgs::LaserScan::ConstPtr> m_laserLayers;
  std::vector<int> m_map_layers;

  /**
   * This map stores which map layers are enabled and which are disabled
   */
  std::map<int, bool> m_MapVisibility;

  // sizes of the last slam map
  bool m_got_transform;

  MaskingManager* m_MaskingManager;

  /** map publisher */
  ros::Publisher m_MapPublisher;
};
#endif
