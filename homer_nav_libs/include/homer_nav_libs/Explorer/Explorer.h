#ifndef EXPLORER_H
#define EXPLORER_H

#include <geometry_msgs/Pose.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <sstream>
#include <vector>

#include <homer_nav_libs/Explorer/GridMap.h>
#include <homer_nav_libs/tools.h>

namespace ExplorerConstants
{
static int8_t UNKNOWN;
static const int8_t NOT_SEEN_YET = -1;
static const double MAX_DISTANCE = DBL_MAX;
static const double MAX_COST = DBL_MAX;
static const int OBSTACLE = INT_MAX;
}

/**
 * @class  Explorer
 * @author Malte Knauf, Stephan Wirth, David Gossow (RX)
 * @brief  Path planning & exploration class
 *
 * Usage:
 *
 * - Call setOccupancyMap() to set the base map for path finding.
 * - Set a start point by calling setStart()
 *
 * - For path planning:
 *   +Choose a target by calling setTarget()
 *   +To correct a target to the nearest approachable position,
 *    call getNearestAccessibleTarget
 *   +Call getPathTransformPath()
 *
 *  -For exploration:
 *   +Call resetExploration()
 *   +Call getExplorationTransformPath()
 *   +The calculated target is the last element in the returned path
 *
 * - Call sampleWaypointsFromPath() to extract waypoints from a calculated path
 *
 * This class uses a couple of "maps" for computation and storing data:
 *
 * - m_OccupancyMap stores the occupancy probabilities in double values. A value
 * of 100 means
 *       totally occupied, 0 totally free.
 * - m_ObstacleDistanceMap stores in each cell the distance (one unit = one
 * cell) to the nearest obstacle.
 *       This map is computed by an eucledian distance transformation from
 * m_OccupancyMap.
 * - m_FrontierMap is a bool map which has 1 in frontier cells and 0 in all
 * others. A frontier
 *       is defined as a free cell that has one of its four direct neighbours in
 * unknown space and is "safe" for
 *       the robot (m_ObstacleDistanceMap is used for that).
 * - m_DrivingDistanceMap is a double map that stores for each cell the distance
 * to m_Start. It is computed
 *       by a flood-fill (seed-fill) algorithm. The values are therefor only an
 * approximation and not exact.
 *       m_DrivingDistanceMap is used to search the nearest frontier when
 * requesting an auto target.
 * - m_TargetMap is a double map that stores for each cell the distance to
 * m_Target. It is computed
 *       like m_DrivingDistanceMap. This map is used as heuristic for the
 * A*-Pathfinding algorithm.
 * - m_NavigationMap is used to mark the cells that are touched by the
 * A*-Pathfinding algorithm.
 *
 *
 * The coordinate system and units that are used in this class are based on map
 * cells.
 * @see GridMap
 *
 */
class Explorer
{
public:
  /**
   * @brief Default constructor.
   * @param minAllowedObstacleDistance,maxAllowedObstacleDistance Range of
   * allowed distances to next obstacle [Pixels]
   * @param minSafeObstacleDistance,maxSafeObstacleDistance Range of distances
   * to next obstacle considered as safe [Pixels]
   * @param safePathWeight Weight for safer path
   */
  Explorer(double minAllowedObstacleDistance, double maxAllowedObstacleDistance,
           double minSafeObstacleDistance, double maxSafeObstacleDistance,
           double safePathWeight, double frontierSafenessFactor = 1.0,
           int unknownThreshold = 50);

  /**
   * @brief Destructor deletes all dynamically allocated memory used by the
   * maps
   */
  ~Explorer();

  void setUnknownThreshold(int unknownTresh);
  void setAllowedObstacleDistance(double min, double max);
  void setSafeObstacleDistance(double min, double max);
  void setFrontierSafenessFactor(double frontierSafenessFactor);
  void setSafePathWeight(double weight);
  /**
   * @brief Copies and sets the occupancy map.
   * @param width Width of the map
   * @param height Height of the map
   * @param origin Real-world pose of the cell (0,0) in the map
   * @param data GridMap-data (occupancy probabilities: 0 = free, 100 =
   * occupied) of size width * height
   */
  void setOccupancyMap(int width, int height, geometry_msgs::Pose origin,
                       int8_t* mapData);
  void setOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr& cmap);

  /** only update occupied areas in current occupancy map */
  void updateObstacles(int width, int height, geometry_msgs::Pose origin,
                       int8_t* mapData);

  /**
   * @brief Sets the start position for the path finding algorithm.
   * m_Start is set to the given value.
   * If startPixel lies outside the map, m_Start remains untouched.
   * @param startPixel Start position for path finding in pixel (map-)
   * coordinates.
   */
  void setStart(Eigen::Vector2i start);

  /**
   * @brief Resets the internal state of the exploration mode.
   * Sets m_DesiredDistance to 0, such that getExplorationTransformPath()
   * triggers
   * a frontier exploration if there is no prior call of setTarget(point,
   * distance).
   * Call this method once before every exploration.
   */
  void resetExploration();

  /**
   * Sets the target position for path finding. m_Target is set to the given
   * value.
   * If endPixel lies outside of the map, m_Target remains untouched.
   * computeTargetDistanceMap() is called at the end of this method. m
   * @param targetPixel Target to reach from startPixel
   */
  void setTarget(Eigen::Vector2i targetPixel);

  /**
   * Sets the target region for path finding. m_ExplorationMap is set to the
   * given region.
   * If targetPixel lies outside of the map, the exploration map is set empty.
   * @param targetPixel Center of the target region to reach from startPixel
   * @param radius Radius of the target region in pixels
   */
  void setTarget(Eigen::Vector2i targetPixel, int radius);

  /**
   * @brief find the nearest position to target that is approachble from the
   * start position
   */
  Eigen::Vector2i getNearestAccessibleTarget(Eigen::Vector2i target);

  /**
   * @brief find the nearest position to target surpassing the minimum
   * obstacle distance
   */
  Eigen::Vector2i getNearestWalkablePoint(Eigen::Vector2i target);

  /**
   * @brief Returns the map-coordinates of the nearest frontier to m_Start.
   * Uses m_DrivingDistanceMap and m_ObstacleDistanceMap. If there is no
   * frontier left,
   * nextFrontier remains untouched.
   * @param[out] nextFrontier Nearest frontier in map-coordinates.
   * @return true if frontier found and stored in nextFrontier, false if no
   * frontier found (nextFrontier
   *         remains untouched).
   */
  bool getNearestFrontier(Eigen::Vector2i& nextFrontier);

  /**
   * Computes the path from m_Start to m_Target with path transform.
   * The result is returned. If the returned vector contains no elements,
   * there is no path.
   * @return vector with path points
   */
  std::vector<Eigen::Vector2i> getPath(bool& success);

  /**
   * Computes the path from m_Start to the next frontier using exploration
   * transform.
   * The result is returned. If the returned vector contains no elements,
   * there is no path.
   * @return vector with path points
   */
  std::vector<Eigen::Vector2i> getExplorationTransformPath(bool& success);

  /**
   * @brief Returns a version of the path that contains less vertices.
   * @note  The nearer the next obstacle, the more waypoints are created.
   * @param path List of vertices to be simplified
   * @param treshold[0..1] a lower threshold results in more waypoints
   * (default:1.0)
   * @return Vector of (sampled) waypoints.
   */
  std::vector<Eigen::Vector2i> sampleWaypointsFromPath(
      std::vector<Eigen::Vector2i> path, float threshold = 1.0);

  /**
   * Getters for the different transforms (see constructor for description)
   */
  GridMap<int8_t>* getOccupancyMap();
  GridMap<double>* getObstacleTransform();
  GridMap<double>* getCostTransform();
  GridMap<bool>* getTargetMap();
  GridMap<double>* getDrivingDistanceTransform();
  GridMap<double>* getTargetDistanceTransform();
  GridMap<double>* getPathTransform();
  GridMap<double>* getExplorationTransform();

  /**
   * @return Start position
   */
  Eigen::Vector2i getStart() const;

  /**
   * @return Target position
   */
  Eigen::Vector2i getTarget() const;

private:
  /** @brief Delete the given map and set pointer to 0 */
  template <class T>
  void releaseMap(GridMap<T>*& map)
  {
    if (map)
    {
      delete map;
      map = 0;
    }
  }

  /** @brief Delete and re-create given map */
  template <class T>
  void resetMap(GridMap<T>*& map)
  {
    if (!m_OccupancyMap)
    {
      ROS_ERROR("Occupancy map is missing.");
      return;
    }
    releaseMap(map);
    map = new GridMap<T>(m_OccupancyMap->width(), m_OccupancyMap->height());
  }

  /**
   * @return true if the robot can stand on the given position without
   * touching an obstacle, false otherwise
   * @warning Call computeWalkableMaps before
   */
  inline bool isWalkable(int x, int y) const
  {
    return (
        (m_OccupancyMap->getValue(x, y) <= ExplorerConstants::UNKNOWN) &&
        (m_ObstacleTransform->getValue(x, y) > m_MinAllowedObstacleDistance));
  }

  /**
   * @return true if point is approachable from the current start position,
   * false otherwise.
   * @warning m_OccupancyMap, m_ObstacleTransform and
   * m_DrivingDistanceTransform have to be present!
   * @warning Call computeApproachableMaps before
   */
  inline bool isApproachable(int x, int y) const
  {
    return (m_DrivingDistanceTransform->getValue(x, y) <
            ExplorerConstants::MAX_DISTANCE);
  }

  /** @brief Releases all memory of the member maps */
  void releaseMaps();

  /**
    * @brief Helper function for computeDistanceTransformation.
    * @param f 1D-Array for distance transformation
    * @param n Number of elements in f
    * @return Distance transformation of f
    */
  double* distanceTransform1D(double* f, int n);

  /**
   * @brief Fills the given map from given start point with distance values to
   * this point.
   * The filling will only be performed on cells that are marked as free in
   * m_OccupancyMap and
   * that have an obstacle distance value between m_MinimumObstacleDistance
   * and m_MaximumObstacleDistance.
   * The map that is passed as argument will be fully overwritten by this
   * function.
   * @param map GridMap to fill
   * @param start Start point for the fill algorithm
   */
  void distanceFloodFill(GridMap<double>* map, Eigen::Vector2i start);

  /** @brief Compute map needed for path calculation */
  void computePathTransform();

  /** @brief Compute map needed for exploration path calculation */
  void computeExplorationTransform();

  /** @brief Compute the distances to the next obstacle with eucledian
   * distance transform from m_OccupancyMap. */
  void computeObstacleTransform();

  /** @brief Compute cost function based on obstacle transform */
  void computeCostTransform();

  /** @brief Compute the frontiers between free and unknown space. Depends on
   * OccupancyMap and ObstacleTransform. */
  void computeFrontierMap();

  /** @brief Compute the target region (a circle of radius m_DesiredDistance
   * around m_Target). */
  void computeRegionMap();

  /** @brief Compute the target map, which is either a frontier map or a
   * region map. */
  void computeTargetMap();

  /** @brief Compute a map of driving distances from the start point */
  void computeDrivingDistanceTransform();

  /** @brief Compute a map of driving distances to the target point */
  void computeTargetDistanceTransform();

  /** @brief Compute maps needed for isWalkable */
  void computeWalkableMaps();

  /** @brief Compute maps needed for isApproachable */
  void computeApproachableMaps();

  /** @brief Start point for the way search algorithm. */
  Eigen::Vector2i m_Start;

  /** @brief Target for the way search algorithm */
  Eigen::Vector2i m_Target;

  /** @brief Desired distance to target in pixels */
  int m_DesiredDistance;

  /** @brief Occupancy map */
  GridMap<int8_t>* m_OccupancyMap;

  /** @see computeObstacleTransform */
  GridMap<double>* m_ObstacleTransform;

  /** @see computeCostTransform */
  GridMap<double>* m_CostTransform;

  /** @see computeTargetMap */
  GridMap<bool>* m_TargetMap;

  /** computeDrivingDistanceTransform */
  GridMap<double>* m_DrivingDistanceTransform;

  /** @see computeTargetDistanceTransform */
  GridMap<double>* m_TargetDistanceTransform;

  /** @see computePathTransform */
  GridMap<double>* m_PathTransform;

  /** @see computeExplorationTransform */
  GridMap<double>* m_ExplorationTransform;

  /** @see constructor */
  double m_MinAllowedObstacleDistance;
  double m_MaxAllowedObstacleDistance;

  double m_MinSafeObstacleDistance;
  double m_MaxSafeObstacleDistance;

  /**
   * Weight for safer path
   */
  double m_SafePathWeight;

  /**
   * Factor for minObstacleDistance that determines if a frontier pixel is
   * valid
   */
  double m_FrontierSafenessFactor;

  /**
   * Real-world pose of the point (0,0) in the map
   */
  geometry_msgs::Pose m_Origin;
};

#endif
