#ifndef TOOLS_H
#define TOOLS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <vector>

/**
 * @author Malte Knauf (2014)
 * Convenience functions that are often used in the mapping and navigation
 * process
 */
namespace map_tools {

/**
 * @brief Converts a point p in world frame /map to the respective cell position
 * in the map
 * @param p Point in world frame
 * @param origin Origin of the map
 * @param resolution Resolution of the map
 * @return Cell position of the point
 */
Eigen::Vector2i toMapCoords(geometry_msgs::Point p, geometry_msgs::Pose origin,
                            float resolution) {
    int x_idx = (p.x - origin.position.x) / resolution + 0.51;
    int y_idx = (p.y - origin.position.y) / resolution + 0.51;
    Eigen::Vector2i ret(x_idx, y_idx);
    return ret;
}

/**
 * @brief Converts a point p in world frame /map to the respective cell position
 * in the map
 * @param p Point in world frame
 * @param origin Origin of the map
 * @param resolution Resolution of the map
 * @return Cell position of the point
 */
Eigen::Vector2i toMapCoords(const geometry_msgs::Point p,
                            const nav_msgs::OccupancyGrid::ConstPtr& cmap) {
    int x_idx =
        (p.x - cmap->info.origin.position.x) / cmap->info.resolution + 0.51;
    int y_idx =
        (p.y - cmap->info.origin.position.y) / cmap->info.resolution + 0.51;
    Eigen::Vector2i ret(x_idx, y_idx);
    return ret;
}

/**
 * @brief Converts the cell position of a point to its respective position in
 * the world frame
 * @param idx Cell position of the point
 * @param origin Origin of the map
 * @param resolution Resolution of the map
 * @return Point in world frame
 */
geometry_msgs::Point fromMapCoords(
    const Eigen::Vector2i idx, const nav_msgs::OccupancyGrid::ConstPtr& cmap) {
    geometry_msgs::Point ret;
    ret.x =
        cmap->info.origin.position.x + (idx.x() - 0.5) * cmap->info.resolution;
    ret.y =
        cmap->info.origin.position.y + (idx.y() - 0.5) * cmap->info.resolution;
    return ret;
}

/**
 * @brief Converts the cell position of a point to its respective position in
 * the world frame
 * @param idx Cell position of the point
 * @param origin Origin of the map
 * @param resolution Resolution of the map
 * @return Point in world frame
 */
geometry_msgs::Point fromMapCoords(Eigen::Vector2i idx,
                                   geometry_msgs::Pose origin,
                                   float resolution) {
    geometry_msgs::Point ret;
    ret.x = origin.position.x + (idx.x() - 0.5) * resolution;
    ret.y = origin.position.y + (idx.y() - 0.5) * resolution;
    return ret;
}

/**
 * @brief Converts the QT pixel position of a point to its respective position
 * in the world frame
 * @param idx Cell position of the point
 * @param origin Origin of the map
 * @param resolution Resolution of the map
 * @return Point in world frame
 */
geometry_msgs::Point qtFromMapCoords(Eigen::Vector2i idx,
                                     geometry_msgs::Pose origin,
                                     float resolution) {
    geometry_msgs::Point ret;
    ret.x = -(origin.position.x + idx.y()) * resolution;
    ret.y = -(origin.position.y + idx.x()) * resolution;
    return ret;
}

/**
 * @brief map_index returns for a given point in the map real-world frame the
 * respective index in the map
 * @param p Point in the real-world frame (usually the frame /map or /world)
 * @param origin Pose of the point (0,0) of the map in the real-world frame
 * @param width Width of the map
 * @param resolution Resolution in meters/cell of the map
 * @return index of point in the map
 */
int map_index(geometry_msgs::Point p, geometry_msgs::Pose origin, float width,
              float resolution) {
    return (int)(width * ((p.y - origin.position.y) / resolution + 0.51) +
                 ((p.x - origin.position.x) / resolution + 0.51));
}

/**
 * @brief point_in_map returns true if given point is in the map. False
 * otherwise
 * @param p Point in the real-world frame (usually the frame /map or /world)
 * @param origin Pose of the point (0,0) of the map in the real-world frame
 * @param width Width of the map
 * @param resolution Resolution in meters/cell of the map
 * @return true or false
 */
bool point_in_map(geometry_msgs::Point p, geometry_msgs::Pose origin,
                  float width, float resolution) {
    int x_idx = (p.x - origin.position.x) / resolution + 0.51;
    int y_idx = (p.y - origin.position.y) / resolution + 0.51;
    if (x_idx < 0 || y_idx < 0 || x_idx >= width || y_idx >= width)
        return false;
    return true;
}

/**
 * @brief transformPoint wrapper to transform points between coordinate frames
 * @param point input point in from_frame
 * @param listener transform listener
 * @param from_frame input frame
 * @param to_frame output frame
 * @return transformed point in to_frame
 */
geometry_msgs::Point transformPoint(geometry_msgs::Point point,
                                    tf::TransformListener& listener,
                                    std::string from_frame,
                                    std::string to_frame,
                                    ros::Time stamp = ros::Time(0)) {
    geometry_msgs::PointStamped pin;
    geometry_msgs::PointStamped pout;
    pin.header.frame_id = from_frame;
    pin.point = point;
    try {
        listener.transformPoint(to_frame, stamp, pin, "/map", pout);
        return pout.point;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point point,
                                    tf::StampedTransform transform) {
    geometry_msgs::Point point_out;
    tf::Vector3 pin;
    tf::Vector3 pout;
    pin.setX(point.x);
    pin.setY(point.y);
    pin.setZ(point.z);

    pout = transform * pin;

    point_out.x = pout.x();
    point_out.y = pout.y();

    return point_out;
}
geometry_msgs::Point transformPoint(const geometry_msgs::Point point,
                                    tf::Transform transform) {
    geometry_msgs::Point point_out;
    tf::Vector3 pin;
    tf::Vector3 pout;
    pin.setX(point.x);
    pin.setY(point.y);
    pin.setZ(point.z);

    pout = transform * pin;

    point_out.x = pout.x();
    point_out.y = pout.y();
    point_out.z = pout.z();

    return point_out;
}

/**
 * @brief transformPoint wrapper to transform points between coordinate frames
 * at specific time
 * @param point input point in from_frame
 * @param listener transform listener
 * @param Time to look for transform
 * @param from_frame input frame
 * @param to_frame output frame
 * @return transformed point in to_frame
 */
geometry_msgs::Point transformPoint(geometry_msgs::Point point,
                                    tf::TransformListener& listener,
                                    const ros::Time& time,
                                    std::string from_frame,
                                    std::string to_frame) {
    geometry_msgs::PointStamped pin;
    geometry_msgs::PointStamped pout;
    pin.header.frame_id = from_frame;
    pin.point = point;
    try {
        listener.transformPoint(to_frame, time, pin, "/map", pout);
        return pout.point;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}

/**
 * @brief laser_range_to_point converts a single given laser scan range in polar
 * coordinates
 *          to the respective point in euclidean coordinates in the target frame
 * @param laser_range range of the laser point to convert
 * @param index
 * @param start_angle
 * @param angle_step
 * @param listener
 * @param from_frame
 * @param to_frame
 * @return
 */
geometry_msgs::Point laser_range_to_point(
    float laser_range, int index, float start_angle, float angle_step,
    tf::TransformListener& listener, std::string from_frame,
    std::string to_frame, ros::Time stamp = ros::Time(0), float time_inc = 0) {
    float alpha = start_angle + index * angle_step;
    geometry_msgs::PointStamped pin;
    geometry_msgs::PointStamped pout;
    pin.header.frame_id = from_frame;
    pin.point.x = cos(alpha) * laser_range;
    pin.point.y = sin(alpha) * laser_range;

    try {
        listener.transformPoint(to_frame, stamp, pin, "/map", pout);
        return pout.point;
    } catch (tf::TransformException ex) {
        // ROS_ERROR("%s",ex.what());
    }
}

/**
 * @brief laser_ranges_to_points converts a given laser scan in polar
 * coordinates
 *          to the respective points in euclidean coordinates in the target
 * frame
 * @param laser_data laser data ranges
 * @param start_angle angle of the first measurement
 * @param angle_step angle increment between two consecutive laser measurements
 * @param range_min minimum valid range
 * @param range_max maximum valid range
 * @return vector containing the laser measurements in euclidean points
 */
std::vector<geometry_msgs::Point> laser_ranges_to_points(
    const std::vector<float>& laser_data, float start_angle, float angle_step,
    float range_min, float range_max, tf::TransformListener& listener,
    std::string from_frame, std::string to_frame,
    ros::Time stamp = ros::Time(0), float time_inc = 0) {
    std::vector<geometry_msgs::Point> ret;
    float alpha = start_angle;
    for (int i = 0; i < laser_data.size(); i++) {
        if (laser_data[i] < range_min || laser_data[i] > range_max) {
            alpha += angle_step;
            continue;
        }
        geometry_msgs::Point point;
        point.x = cos(alpha) * laser_data.at(i);
        point.y = sin(alpha) * laser_data.at(i);

        geometry_msgs::PointStamped pin;
        pin.header.frame_id = from_frame;
        pin.point = point;
        geometry_msgs::PointStamped pout;
        try {
            listener.transformPoint(to_frame, stamp, pin, "/map", pout);
            ret.push_back(pout.point);
        } catch (tf::TransformException ex) {
            // ROS_ERROR("%s",ex.what());
        }

        alpha += angle_step;
    }
    return ret;
}

/**
* @brief laser_msg_to_points converts a given laser scan in polar coordinates
*          to the respective points in euclidean coordinates in the target frame
*			at a specific time
* @param scan laser data msg
* @param listener TransformListener
* @param to_frame target frame
* @return vector containing the laser measurements in euclidean points
*/
std::vector<geometry_msgs::Point> laser_msg_to_points(
    const sensor_msgs::LaserScan::ConstPtr& scan,
    tf::TransformListener& listener, std::string to_frame,
    ros::Time stamp = ros::Time(0)) {
    std::vector<geometry_msgs::Point> ret;
    float alpha = scan->angle_min;
    if (!listener.waitForTransform(scan->header.frame_id, to_frame, stamp,
                                   ros::Duration(0.3))) {
        return ret;
    }
    for (int i = 0; i < scan->ranges.size(); i++) {
        if (scan->ranges[i] < scan->range_min ||
            scan->ranges[i] > scan->range_max) {
            alpha += scan->angle_increment;
            continue;
        }
        geometry_msgs::Point point;
        point.x = cos(alpha) * scan->ranges.at(i);
        point.y = sin(alpha) * scan->ranges.at(i);

        geometry_msgs::PointStamped pin;
        pin.header.frame_id = scan->header.frame_id;
        pin.point = point;
        geometry_msgs::PointStamped pout;
        try {
            // listener.transformPoint(to_frame, (stamp + ros::Duration( i *
            // scan->time_increment)), pin, "/map" ,pout);
            listener.transformPoint(to_frame, stamp, pin, "/map", pout);
            ret.push_back(pout.point);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }

        alpha += scan->angle_increment;
    }
    return ret;
}

/**
* @brief get_max_move_distance searches for the nearest values in an defined
* area
* @param vector points: laserPoints in base_link
* @return float distance to nearest Point
*/
float get_max_move_distance(std::vector<geometry_msgs::Point> points,
                            float min_x, float min_y) {
    float minDistance = 30;
    for (unsigned int i = 0; i < points.size(); i++) {
        if (std::fabs(points[i].y) < min_y && points[i].x > min_x) {
            float distance =
                sqrt((points[i].x * points[i].x) + (points[i].y * points[i].y));
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }
    float maxMoveDist = minDistance - min_x;
    if (maxMoveDist < 0) {
        maxMoveDist = 0.0;
    }
    return maxMoveDist;
}

/**
 * @brief Calculates the euclidean distance (in cells) between to points in the
 * map
 * @param a Point a
 * @param b point b
 * @return euclidean distance in cells
 */
double distance(const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
    return sqrt((a.x() - b.x()) * (a.x() - b.x()) +
                (a.y() - b.y()) * (a.y() - b.y()));
}

/**
 * @brief Calculates the euclidean distance (in m) between to points in the
 * world
 * @param a Point a
 * @param b point b
 * @return euclidean distance in m
 */
double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief findValue
 * @param map Pointer to the map to search
 * @param width Width of the map
 * @param height Height of the map
 * @param center_x \__ Center point of circle to search within
 * @param center_y /
 * @param value Value to search for in given map
 * @param radius Radius of the circle
 * @return true if the given value could be found within the given radius around
 * (x,y)
 */
bool findValue(const std::vector<int8_t>* map, int width, int height,
               int center_x, int center_y, unsigned char value, float radius) {
    int start_x = int(center_x - radius);
    int start_y = int(center_y - radius);
    int end_x = int(center_x + radius);
    int end_y = int(center_y + radius);

    if (start_x < 0) {
        start_x = 0;
    }
    if (start_y < 0) {
        start_y = 0;
    }
    if (end_x >= int(width)) {
        end_x = width - 1;
    }
    if (end_y >= int(height)) {
        end_y = height - 1;
    }

    float sqr_radius = radius * radius;

    for (int y = start_y; y <= end_y; y++)
        for (int x = start_x; x <= end_x; x++) {
            if (map->at(x + width * y) > value) {
                float sqr_dist = float(x - center_x) * float(x - center_x) +
                                 float(y - center_y) * float(y - center_y);
                if (sqr_dist <= sqr_radius) {
                    return true;
                }
            }
        }

    return false;
}
}

#endif  // TOOLS_H
