#include <homer_nav_libs/Explorer/Explorer.h>

using namespace std;
using namespace ExplorerConstants;

Explorer::Explorer(double minAllowedObstacleDistance,
                   double maxAllowedObstacleDistance,
                   double minSafeObstacleDistance,
                   double maxSafeObstacleDistance, double safePathWeight,
                   double frontierSafenessFactor, int unknownThreshold) {
    ExplorerConstants::UNKNOWN = unknownThreshold;

    m_MinAllowedObstacleDistance = minAllowedObstacleDistance;
    m_MaxAllowedObstacleDistance = maxAllowedObstacleDistance;

    m_MinSafeObstacleDistance = minSafeObstacleDistance;
    m_MaxSafeObstacleDistance = maxSafeObstacleDistance;

    m_SafePathWeight = safePathWeight;
    m_FrontierSafenessFactor = frontierSafenessFactor;

    m_OccupancyMap = 0;
    m_ObstacleTransform = 0;
    m_CostTransform = 0;
    m_TargetMap = 0;
    m_DrivingDistanceTransform = 0;
    m_TargetDistanceTransform = 0;
    m_PathTransform = 0;
    m_ExplorationTransform = 0;
    m_DesiredDistance = 0;
}

Explorer::~Explorer() {
    releaseMaps();
    releaseMap(m_OccupancyMap);
}

void Explorer::releaseMaps() {
    releaseMap(m_TargetMap);
    releaseMap(m_ObstacleTransform);
    releaseMap(m_CostTransform);
    releaseMap(m_DrivingDistanceTransform);
    releaseMap(m_TargetDistanceTransform);
    releaseMap(m_PathTransform);
    releaseMap(m_ExplorationTransform);
}

// SETTERS
// ////////////////////////////////////////////////////////////////////////////////////////////////

void Explorer::setUnknownThreshold(int unknownTresh) {
    ExplorerConstants::UNKNOWN = unknownTresh;
}

void Explorer::setAllowedObstacleDistance(double min, double max) {
    m_MinAllowedObstacleDistance = min;
    m_MaxAllowedObstacleDistance = max;
    releaseMaps();
}

void Explorer::setSafeObstacleDistance(double min, double max) {
    m_MinSafeObstacleDistance = min;
    m_MaxSafeObstacleDistance = max;
    releaseMaps();
}

void Explorer::setSafePathWeight(double weight) {
    m_SafePathWeight = weight;
    releaseMaps();
}

void Explorer::setFrontierSafenessFactor(double frontierSafenessFactor) {
    m_FrontierSafenessFactor = frontierSafenessFactor;
    releaseMaps();
}

void Explorer::setOccupancyMap(int width, int height,
                               geometry_msgs::Pose origin, int8_t* data) {
    if (!data) {
        ROS_ERROR("Received 0-pointer.");
        return;
    }
    releaseMaps();
    releaseMap(m_OccupancyMap);
    // m_OccupancyMap = new GridMap<unsigned char> ( width, height, data,
    // exploredRegion );
    m_OccupancyMap = new GridMap<int8_t>(width, height, data);
    m_Origin = origin;
}

void Explorer::setOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr& cmap) {
    releaseMaps();
    releaseMap(m_OccupancyMap);
    // m_OccupancyMap = new GridMap<unsigned char> ( width, height, data,
    // exploredRegion );
    nav_msgs::OccupancyGrid temp_map = *cmap;
    m_OccupancyMap = new GridMap<int8_t>(cmap->info.width, cmap->info.height,
                                         &(temp_map.data)[0]);
    m_Origin = cmap->info.origin;
}

void Explorer::updateObstacles(int width, int height,
                               geometry_msgs::Pose origin, int8_t* mapData) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }
    if ((width != m_OccupancyMap->width()) ||
        (height != m_OccupancyMap->height())) {
        ROS_ERROR_STREAM("Wrong map size!");
        return;
    }
    for (unsigned i = 0; i < m_OccupancyMap->width() * m_OccupancyMap->height();
         i++) {
        int8_t* myMapData = m_OccupancyMap->getDirectAccess(0, 0);
        if (myMapData[i] != UNKNOWN) {
            myMapData[i] = mapData[i];
        }
    }
    releaseMaps();
}

void Explorer::resetExploration() { m_DesiredDistance = 0; }

void Explorer::setStart(Eigen::Vector2i start) {
    if (!m_OccupancyMap) {
        ROS_ERROR_STREAM("Occupancy map is missing.");
        return;
    }
    if (start.x() <= 1)
    { 
        start.x() = 2;
    }
    if (start.y() <= 1)
    {
        start.y() = 2;
    }
    if (start.x() >= m_OccupancyMap->width() - 1) 
    {
        start.x() = m_OccupancyMap->width() - 2;
    }
    if (start.y() >= m_OccupancyMap->height() - 1)
    {
        start.y() = m_OccupancyMap->height() -2;
    }
    computeWalkableMaps();

    if (!isWalkable(start.x(), start.y())) {
        Eigen::Vector2i correctedStart = getNearestWalkablePoint(start);
        if (!isWalkable(correctedStart.x(), correctedStart.y())) {
            ROS_ERROR_STREAM("No walkable position was found on the map!");
        } else {
            ROS_INFO_STREAM("Start position " << start.x() << "," << start.y()
                                              << " was corrected to "
                                              << correctedStart.x() << ","
                                              << correctedStart.y());
        }
        m_Start = correctedStart;
        return;
    }
    m_Start = start;
}

Eigen::Vector2i Explorer::getNearestAccessibleTarget(Eigen::Vector2i target) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return target;
    }
    if (target.x() <= 1)
    { 
        target.x() = 2;
    }
    if (target.y() <= 1)
    {
        target.y() = 2;
    }
    if (target.x() >= m_OccupancyMap->width() - 1) 
    {
        target.x() = m_OccupancyMap->width() - 2;
    }
    if (target.y() >= m_OccupancyMap->height() - 1)
    {
        target.y() = m_OccupancyMap->height() -2;
    }
    computeApproachableMaps();
    computeWalkableMaps();
    Eigen::Vector2i correctTarget = target;

    if (!isApproachable(target.x(), target.y())) {
        ROS_INFO_STREAM(
            "target cell in drivingdistancetransform: "
            << m_DrivingDistanceTransform->getValue(target.x(), target.y()));
        ROS_INFO_STREAM("target "
                        << target
                        << " is not approachable. Correcting target...");
        int minSqrDist = INT_MAX;
        for (int x = 0; x < m_ObstacleTransform->width(); x++) {
            for (int y = 0; y < m_ObstacleTransform->height(); y++) {
                bool isSafe =
                    m_ObstacleTransform->getValue(x, y) >
                    m_FrontierSafenessFactor * m_MinAllowedObstacleDistance;
                if (isApproachable(x, y) && isWalkable(x, y) && isSafe) {
                    int xDiff = target.x() - x;
                    int yDiff = target.y() - y;
                    int sqrDist = xDiff * xDiff + yDiff * yDiff;
                    if (sqrDist < minSqrDist) {
                        correctTarget.x() = x;
                        correctTarget.y() = y;
                        minSqrDist = sqrDist;
                    }
                }
            }
        }
    }
    ROS_DEBUG_STREAM("Target position "
                     << target.x() << "," << target.y() << " was corrected to "
                     << correctTarget.x() << "," << correctTarget.y());

    return correctTarget;
}

Eigen::Vector2i Explorer::getNearestWalkablePoint(Eigen::Vector2i target) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return target;
    }
    if (target.x() <= 1)
    { 
        target.x() = 2;
    }
    if (target.y() <= 1)
    {
        target.y() = 2;
    }
    if (target.x() >= m_OccupancyMap->width() - 1) 
    {
        target.x() = m_OccupancyMap->width() - 2;
    }
    if (target.y() >= m_OccupancyMap->height() - 1)
    {
        target.y() = m_OccupancyMap->height() -2;
    }

    computeWalkableMaps();
    Eigen::Vector2i correctTarget = target;

    if (!isWalkable(target.x(), target.y())) {
        int minSqrDist = INT_MAX;
        for (int x = 0; x < m_ObstacleTransform->width(); x++) {
            for (int y = 0; y < m_ObstacleTransform->height(); y++) {
                if (isWalkable(x, y)) {
                    int xDiff = target.x() - x;
                    int yDiff = target.y() - y;
                    int sqrDist = xDiff * xDiff + yDiff * yDiff;
                    if (sqrDist < minSqrDist) {
                        correctTarget.x() = x;
                        correctTarget.y() = y;
                        minSqrDist = sqrDist;
                    }
                }
            }
        }
    }
    ROS_DEBUG_STREAM("Position " << target.x() << "," << target.y()
                                 << " was corrected to " << correctTarget.x()
                                 << "," << correctTarget.y());

    return correctTarget;
}

void Explorer::setTarget(Eigen::Vector2i target) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }
    if (target.x() <= 1)
    { 
        target.x() = 2;
    }
    if (target.y() <= 1)
    {
        target.y() = 2;
    }
    if (target.x() >= m_OccupancyMap->width() - 1) 
    {
        target.x() = m_OccupancyMap->width() - 2;
    }
    if (target.y() >= m_OccupancyMap->height() - 1)
    {
        target.y() = m_OccupancyMap->height() -2;
    }
    computeApproachableMaps();
    if (!isApproachable(target.x(), target.y())) {
        ROS_WARN(
            "Target position is not approachable. Path computation will "
            "possibly fail.");
    }
    m_Target = target;
    m_DesiredDistance = 0;
}

void Explorer::setTarget(Eigen::Vector2i target, int desiredDistance) {
    ROS_ERROR_STREAM("setTarget still in use!!");
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }

    if (desiredDistance < 1) {
        setTarget(target);
        return;
    }

    if (target.x() + desiredDistance <= 1)
    { 
        target.x() = 2;
    }
    if (target.y() + desiredDistance <= 1)
    {
        target.y() = 2;
    }
    if (target.x() - desiredDistance >= m_OccupancyMap->width() - 1) 
    {
        target.x() = m_OccupancyMap->width() - 2;
    }
    if (target.y() - desiredDistance >= m_OccupancyMap->height() - 1)
    {
        target.y() = m_OccupancyMap->height() -2;
    }
    computeApproachableMaps();
    // TODO: check if region is approachable
    m_Target = target;
    m_DesiredDistance = desiredDistance;
}

// GETTERS
// ////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2i Explorer::getStart() const { return m_Start; }

Eigen::Vector2i Explorer::getTarget() const { return m_Target; }

GridMap<int8_t>* Explorer::getOccupancyMap() { return m_OccupancyMap; }

GridMap<double>* Explorer::getObstacleTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computeObstacleTransform();
    return m_ObstacleTransform;
}

GridMap<double>* Explorer::getCostTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computeCostTransform();
    return m_CostTransform;
}

GridMap<bool>* Explorer::getTargetMap() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }

    computeTargetMap();
    return m_TargetMap;
}

GridMap<double>* Explorer::getDrivingDistanceTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computeDrivingDistanceTransform();
    return m_DrivingDistanceTransform;
}

GridMap<double>* Explorer::getTargetDistanceTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computeTargetDistanceTransform();
    return m_TargetDistanceTransform;
}

GridMap<double>* Explorer::getPathTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computePathTransform();
    return m_PathTransform;
}

GridMap<double>* Explorer::getExplorationTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return 0;
    }
    computeExplorationTransform();
    return m_ExplorationTransform;
}

// MAP GENERATION
// ////////////////////////////////////////////////////////////////////////////////////////////////7

void Explorer::computeApproachableMaps() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }
    computeDrivingDistanceTransform();
}

void Explorer::computeWalkableMaps() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }
    computeObstacleTransform();
}

void Explorer::computeDrivingDistanceTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }

    if (m_DrivingDistanceTransform) {
        return;
    }

    ROS_DEBUG("Computing drivingDistanceTransform...");
    resetMap(m_DrivingDistanceTransform);
    distanceFloodFill(m_DrivingDistanceTransform, m_Start);
}

void Explorer::computeTargetDistanceTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }

    if (m_TargetDistanceTransform) {
        return;
    }

    ROS_DEBUG("Computing targetDistanceTransform...");
    resetMap(m_TargetDistanceTransform);
    distanceFloodFill(m_TargetDistanceTransform, m_Target);
}

void Explorer::computeRegionMap() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }

    resetMap(m_TargetMap);
    ROS_DEBUG("Computing target region map...");

    m_TargetMap->fill(false);
    const int desiredDistanceSquared = m_DesiredDistance * m_DesiredDistance;
    int height = m_OccupancyMap->height();
    int width = m_OccupancyMap->width();

    // draw a circle onto the ExplorationMap
    const int firstX = m_Target.x() - m_DesiredDistance <= 1
                           ? 2
                           : m_Target.x() - m_DesiredDistance;
    const int firstY = m_Target.y() - m_DesiredDistance <= 1
                           ? 2
                           : m_Target.y() - m_DesiredDistance;
    const int lastX = m_Target.x() + m_DesiredDistance >= width - 1
                          ? width - 2
                          : m_Target.x() + m_DesiredDistance;
    const int lastY = m_Target.y() + m_DesiredDistance >= height - 1
                          ? height - 2
                          : m_Target.y() + m_DesiredDistance;

    for (int y = firstY; y <= lastY; ++y) {
        for (int x = firstX; x <= lastX; ++x) {
            const int dx = x - m_Target.x();
            const int dy = y - m_Target.y();

            if (dx * dx + dy * dy <= desiredDistanceSquared) {
                m_TargetMap->setValue(x, y, true);
            }
        }
    }
}

void Explorer::computeFrontierMap() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Occupancy map is missing.");
        return;
    }

    // if ( m_FrontierMap ) { return; }

    resetMap(m_TargetMap);

    ROS_DEBUG("Computing frontier map...");
    m_TargetMap->fill(0);
    // extract borders
    for (int y = 1; y < m_OccupancyMap->height() - 1; y++) {
        for (int x = 1; x < m_OccupancyMap->width() - 1; x++) {
            int value = m_OccupancyMap->getValue(x, y);
            int value_u = m_OccupancyMap->getValue(x, y - 1);
            int value_d = m_OccupancyMap->getValue(x, y + 1);
            int value_l = m_OccupancyMap->getValue(x - 1, y);
            int value_r = m_OccupancyMap->getValue(x + 1, y);
            bool isFree = value < UNKNOWN && value != NOT_SEEN_YET;
            bool upUnknown = (value_u == UNKNOWN || value_u == NOT_SEEN_YET);
            bool downUnknown = (value_d == UNKNOWN || value_u == NOT_SEEN_YET);
            bool leftUnknown = (value_l == UNKNOWN || value_u == NOT_SEEN_YET);
            bool rightUnknown = (value_r == UNKNOWN || value_u == NOT_SEEN_YET);
            bool hasUnknownNeighbour =
                upUnknown || downUnknown || leftUnknown || rightUnknown;
            bool isSafe =
                m_ObstacleTransform->getValue(x, y) >
                m_FrontierSafenessFactor * m_MinAllowedObstacleDistance;
            if (isFree && hasUnknownNeighbour && isSafe) {
                m_TargetMap->setValue(x, y, 1);
            } else {
                m_TargetMap->setValue(x, y, 0);
            }
        }
    }
}

void Explorer::computeTargetMap() {
    ROS_ERROR_STREAM("target Map shouldn't be used anymore!");
    if (m_DesiredDistance < 1) {
        computeFrontierMap();
    } else {
        computeRegionMap();
    }
}

void Explorer::computeObstacleTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return;
    }

    if (m_ObstacleTransform) {
        return;
    }

    resetMap(m_ObstacleTransform);

    ROS_DEBUG("Computing obstacle transform...");
    for (int x = 0; x < m_ObstacleTransform->width(); x++) {
        for (int y = 0; y < m_ObstacleTransform->height(); y++) {
            if (m_OccupancyMap->getValue(x, y) > UNKNOWN ||
                m_OccupancyMap->getValue(x, y) == NOT_SEEN_YET) {
                m_ObstacleTransform->setValue(x, y, 0);  // Obstacle
            } else {
                m_ObstacleTransform->setValue(x, y, INT_MAX);  // Free
            }
        }
    }

    int width = m_ObstacleTransform->width();
    int height = m_ObstacleTransform->height();
    double* f = new double[width > height ? width : height];

    // transform along columns
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            // copy column
            f[y] = m_ObstacleTransform->getValue(x, y);
        }
        // 1-D transform of column
        double* d = distanceTransform1D(f, height);
        // copy transformed 1-D to output image
        for (int y = 0; y < height; y++) {
            m_ObstacleTransform->setValue(x, y, d[y]);
        }
        delete[] d;
    }

    // transform along rows
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            f[x] = m_ObstacleTransform->getValue(x, y);
        }
        double* d = distanceTransform1D(f, width);
        for (int x = 0; x < width; x++) {
            m_ObstacleTransform->setValue(x, y, d[x]);
        }
        delete[] d;
    }
    delete f;

    // take square roots
    for (int y = 0; y < m_ObstacleTransform->height(); y++) {
        for (int x = 0; x < m_ObstacleTransform->width(); x++) {
            if (isWalkable(x, y)) {
                float value = sqrt(m_ObstacleTransform->getValue(x, y));
                m_ObstacleTransform->setValue(x, y, value);
            }
        }
    }
}

void Explorer::computeCostTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return;
    }

    if (m_CostTransform) {
        return;
    }

    computeObstacleTransform();
    computeApproachableMaps();

    resetMap(m_CostTransform);
    m_CostTransform->fill(ExplorerConstants::MAX_COST);

    for (unsigned y = 0; y < m_CostTransform->height(); y++) {
        for (unsigned x = 0; x < m_CostTransform->width(); x++) {
            if (!isApproachable(x, y)) {
                continue;
            }
            double dist = m_ObstacleTransform->getValue(x, y);
            double cost = 0;
            if (dist < m_MinSafeObstacleDistance) {
                cost = m_MinSafeObstacleDistance - dist;
            }
            //      if ( dist > m_MaxSafeObstacleDistance ) {
            //        cost = dist - m_MaxSafeObstacleDistance;
            //      }
            m_CostTransform->setValue(x, y, cost * cost);
        }
    }
}

void Explorer::computePathTransform() {
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return;
    }

    if (m_PathTransform) {
        return;
    }

    computeObstacleTransform();
    computeCostTransform();

    resetMap(m_PathTransform);

    ROS_DEBUG("Computing path transform...");
    GridMap<double>* map = m_PathTransform;
    int width = map->width();
    int height = map->height();
    double maxDistance = MAX_DISTANCE;
    map->fill(maxDistance);

    int fromX = m_Target.x();
    int fromY = m_Target.y();
    map->setValue(fromX, fromY, 0);

    queue<int> xQueue;
    queue<int> yQueue;
    xQueue.push(fromX + 1);
    yQueue.push(fromY);
    xQueue.push(fromX - 1);
    yQueue.push(fromY);
    xQueue.push(fromX);
    yQueue.push(fromY - 1);
    xQueue.push(fromX);
    yQueue.push(fromY + 1);
    int xVal, yVal;
    while (!xQueue.empty()) {
        xVal = xQueue.front();
        yVal = yQueue.front();
        xQueue.pop();
        yQueue.pop();
        if (xVal > 0 && xVal < width - 1 && yVal > 0 && yVal < height - 1 &&
            isWalkable(xVal, yVal)) {
            float value = map->getValue(xVal, yVal);
            float value_u = map->getValue(xVal, yVal - 1) + 1;
            float value_d = map->getValue(xVal, yVal + 1) + 1;
            float value_l = map->getValue(xVal - 1, yVal) + 1;
            float value_r = map->getValue(xVal + 1, yVal) + 1;

            float value_ur = map->getValue(xVal + 1, yVal - 1) + 1.4142;
            float value_ul = map->getValue(xVal - 1, yVal - 1) + 1.4142;
            float value_ll = map->getValue(xVal - 1, yVal + 1) + 1.4142;
            float value_lr = map->getValue(xVal + 1, yVal + 1) + 1.4142;

            float min1 = value_u < value_d ? value_u : value_d;
            float min2 = value_l < value_r ? value_l : value_r;
            float min3 = value_ur < value_ul ? value_ur : value_ul;
            float min4 = value_ll < value_lr ? value_ll : value_lr;
            float min12 = min1 < min2 ? min1 : min2;
            float min34 = min3 < min4 ? min3 : min4;
            float min = min12 < min34 ? min12 : min34;
            float newVal =
                min + m_SafePathWeight * m_CostTransform->getValue(xVal, yVal);
            if (value > newVal) {
                map->setValue(xVal, yVal, newVal);
                if (map->getValue(xVal, yVal + 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal, yVal - 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal) > newVal + 1) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal - 1, yVal) > newVal + 1) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal + 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal - 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal - 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal + 1);
                }
            }
        }
    }
}

void Explorer::computeExplorationTransform() {
    ROS_ERROR_STREAM("Exploration Transform shouldn't be used!");
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return;
    }

    if (m_ExplorationTransform) {
        return;
    }

    ROS_DEBUG_STREAM("computeExplorationTransform: before obstacle transform");
    computeObstacleTransform();
    ROS_DEBUG_STREAM("computeExplorationTransform: before cost transform");
    computeCostTransform();
    ROS_DEBUG_STREAM("computeExplorationTransform: before target map");
    computeTargetMap();
    ROS_DEBUG_STREAM("computeExplorationTransform: before walkable maps");
    computeWalkableMaps();
    ROS_DEBUG_STREAM(
        "computeExplorationTransform: before exploration transform");
    resetMap(m_ExplorationTransform);

    ROS_DEBUG("Computing exploration transform...");
    GridMap<double>* map = m_ExplorationTransform;
    int width = map->width();
    int height = map->height();
    double maxDistance = MAX_DISTANCE;
    map->fill(maxDistance);
    queue<int> xQueue;
    queue<int> yQueue;
    // fill seeds: Mark the frontiers as targets
    ROS_DEBUG_STREAM("computeExplorationTransform: before first loop");
    for (int y = 0; y < m_TargetMap->height(); y++) {
        for (int x = 0; x < m_TargetMap->width(); x++) {
            if (m_TargetMap->getValue(x, y) == 1) {
                map->setValue(x, y, 0);
                xQueue.push(x + 1);
                yQueue.push(y);
                xQueue.push(x - 1);
                yQueue.push(y);
                xQueue.push(x);
                yQueue.push(y - 1);
                xQueue.push(x);
                yQueue.push(y + 1);
            }
        }
    }
    ROS_DEBUG_STREAM("computeExplorationTransform: After first looop");
    // Now go through the coordinates in the queue
    int xVal, yVal;
    ROS_DEBUG_STREAM("computeExplorationTransform: before while loop");
    while (!xQueue.empty()) {
        xVal = xQueue.front();
        yVal = yQueue.front();
        xQueue.pop();
        yQueue.pop();
        if (xVal > 0 && xVal < width - 1 && yVal > 0 && yVal < height - 1 &&
            isWalkable(xVal, yVal)) {
            // Get own cost and the cost of the 8 neighbor cells (neighbors plus
            // the cost to go there)
            float value = map->getValue(xVal, yVal);
            float value_u = map->getValue(xVal, yVal - 1) + 1;
            float value_d = map->getValue(xVal, yVal + 1) + 1;
            float value_l = map->getValue(xVal - 1, yVal) + 1;
            float value_r = map->getValue(xVal + 1, yVal) + 1;
            float value_ur = map->getValue(xVal + 1, yVal - 1) + 1.4142;
            float value_ul = map->getValue(xVal - 1, yVal - 1) + 1.4142;
            float value_ll = map->getValue(xVal - 1, yVal + 1) + 1.4142;
            float value_lr = map->getValue(xVal + 1, yVal + 1) + 1.4142;
            float min1 = value_u < value_d ? value_u : value_d;
            float min2 = value_l < value_r ? value_l : value_r;
            float min3 = value_ur < value_ul ? value_ur : value_ul;
            float min4 = value_ll < value_lr ? value_ll : value_lr;
            float min12 = min1 < min2 ? min1 : min2;
            float min34 = min3 < min4 ? min3 : min4;
            float min = min12 < min34 ? min12 : min34;
            float newVal =
                min + m_SafePathWeight * m_CostTransform->getValue(xVal, yVal);
            if (value > newVal) {
                // Cost is lower then the currently known cost: Reduce cost here
                map->setValue(xVal, yVal, newVal);
                // Add the neighbours that might profit in the queue
                if (map->getValue(xVal, yVal + 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal, yVal - 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal) > newVal + 1) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal - 1, yVal) > newVal + 1) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal + 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal - 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal - 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal + 1);
                }
            }
        }
    }
    ROS_DEBUG_STREAM(
        "computeExplorationTransform: after exploration transform");
}

vector<Eigen::Vector2i> Explorer::sampleWaypointsFromPath(
    std::vector<Eigen::Vector2i> pathPoints, float threshold) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return pathPoints;
    }
    if (pathPoints.size() < 3) {
        return pathPoints;
    }

    computeObstacleTransform();

    vector<Eigen::Vector2i> simplifiedPath;
    simplifiedPath.reserve(pathPoints.size());

    Eigen::Vector2i lastAddedPoint = pathPoints[0];
    simplifiedPath.push_back(lastAddedPoint);

    for (unsigned int i = 1; i < pathPoints.size() - 1; i++) {
        double distanceToNextPoint =
            map_tools::distance(lastAddedPoint, pathPoints.at(i));
        double obstacleDistanceLastAddedPoint = m_ObstacleTransform->getValue(
            lastAddedPoint.x(), lastAddedPoint.y());
        double obstacleDistancePossibleNextPoint =
            m_ObstacleTransform->getValue(pathPoints[i].x(), pathPoints[i].y());
        if ((distanceToNextPoint >=
             obstacleDistanceLastAddedPoint * threshold) ||
            (distanceToNextPoint >=
             obstacleDistancePossibleNextPoint * threshold)) {
            simplifiedPath.push_back(pathPoints[i]);
            lastAddedPoint = pathPoints[i];
        }
    }
    simplifiedPath.push_back(pathPoints[pathPoints.size() - 1]);
    return simplifiedPath;
}

std::vector<Eigen::Vector2i> Explorer::getPath(bool& success) {
    success = false;

    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return vector<Eigen::Vector2i>();
    }

    if (m_DesiredDistance > 0) {
        // we are actually performing an exploration since the target
        // is a region.
        ROS_ERROR_STREAM(
            "Desired Distance > 0: Executing getExplorationTransformPath");
        return getExplorationTransformPath(success);
    }
    ROS_DEBUG_STREAM("Computing Path Transform");
    computePathTransform();
    ROS_DEBUG_STREAM("Finished Path Transform");

    vector<Eigen::Vector2i> path;

    int x = m_Start.x();
    int y = m_Start.y();

    int width = m_OccupancyMap->width();
    int height = m_OccupancyMap->height();

    // special case: start and end point are equal, return single waypoint
    if (map_tools::distance(m_Start, m_Target) < 2.0) {
        success = true;
        path.push_back(Eigen::Vector2i(m_Start.x(), m_Start.y()));
        return path;
    }

    while (x != m_Target.x() || y != m_Target.y()) {
        path.push_back(Eigen::Vector2i(x, y));
        int minPosX = x;
        int minPosY = y;
        double min = m_PathTransform->getValue(x, y);

        if ((x <= 1) || (y <= 1) || (x >= width - 1) || (y >= height - 1)) {
            ROS_ERROR("Out of map bounds");
            return vector<Eigen::Vector2i>();
        }

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                double pt = m_PathTransform->getValue(x + i, y + j);
                if (pt < min) {
                    min = pt;
                    minPosX = x + i;
                    minPosY = y + j;
                }
            }
        }
        if (minPosX == x && minPosY == y) {
            ROS_WARN("Target is unreachable!");
            return vector<Eigen::Vector2i>();
        } else {
            x = minPosX;
            y = minPosY;
        }
    }
    success = true;

    return path;
}

vector<Eigen::Vector2i> Explorer::getExplorationTransformPath(bool& success) {
    success = false;

    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return vector<Eigen::Vector2i>();
    }

    ROS_DEBUG_STREAM("Exploration Transform: Before obstacle transform");
    computeObstacleTransform();
    ROS_DEBUG_STREAM("Exploration Transform: Before exploration transform");
    computeExplorationTransform();
    ROS_DEBUG_STREAM("Exploration Transform: after obstacle transform");

    // check if we are already there
    if (m_TargetMap->getValue(m_Start.x(), m_Start.y())) {
        success = true;
        vector<Eigen::Vector2i> path;
        path.push_back(Eigen::Vector2i(m_Start.x(), m_Start.y()));
        return path;
    }

    int width = m_OccupancyMap->width();
    int height = m_OccupancyMap->height();

    vector<Eigen::Vector2i> path;
    int x = m_Start.x();
    int y = m_Start.y();

    if (m_ObstacleTransform->getValue(x, y) < m_MinAllowedObstacleDistance) {
        // robot got stuck!
        // find way out using ObstacleTransform...
        int maxPosX = x;
        int maxPosY = y;

        if ((x <= 1) || (y <= 1) || (x >= width - 1) || (y >= height - 1)) {
            ROS_ERROR("Out of map bounds");
            return vector<Eigen::Vector2i>();
        }

        while (m_ObstacleTransform->getValue(maxPosX, maxPosY) <
               m_MinAllowedObstacleDistance) {
            double max = m_ObstacleTransform->getValue(x, y);
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    double pt = m_ObstacleTransform->getValue(x + i, y + j);
                    if (pt > max) {
                        max = pt;
                        maxPosX = x + i;
                        maxPosY = y + j;
                    }
                }
            }
            if (maxPosX == x && maxPosY == y)  // no ascentFound
            {
                break;
            } else {
                path.push_back(Eigen::Vector2i(maxPosX, maxPosY));
                x = maxPosX;
                y = maxPosY;
            }
        }
    }
    // now path is "free"
    bool descentFound = true;
    while (descentFound) {
        descentFound = false;
        int minPosX = x;
        int minPosY = y;
        double min = m_ExplorationTransform->getValue(x, y);
        if ((x <= 1) || (y <= 1) || (x >= width - 1) || (y >= height - 1)) {
            ROS_ERROR("Out of map bounds");
            return vector<Eigen::Vector2i>();
        }

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                double pt = m_ExplorationTransform->getValue(x + i, y + j);
                if (pt < min) {
                    min = pt;
                    minPosX = x + i;
                    minPosY = y + j;
                }
            }
        }
        if (minPosX == x && minPosY == y)  // no descentFound
        {
            descentFound = false;
        } else {
            descentFound = true;
            path.push_back(Eigen::Vector2i(minPosX, minPosY));
            x = minPosX;
            y = minPosY;
        }
    }
    success = true;

    ROS_INFO_STREAM("Exploration Transform: End of function");
    return path;

#if 0  
  // START P2AT HACK
  vector< Eigen::Vector2i > newPath;
  for ( unsigned start=0; start<path.size()-1; ++start )
  {
    int maxVal = start+1;
    for ( unsigned end=start+1; end<path.size(); ++end )
    {
      bool ok = true;
      // draw bresenham line and check wether an object is within maximum allowed distance
      // THANKS TO WIKIPEDIA 
      int x, y, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, es, el, err;
      /* Entfernung in beiden Dimensionen berechnen */
      dx = path[end].x() - path[start].x();
      dy = path[end].y() - path[start].y();
      /* Vorzeichen des Inkrements bestimmen */
      incx = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
      incy = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;
      if(dx<0) dx = -dx;
      if(dy<0) dy = -dy;
      /* feststellen, welche Entfernung größer ist */
      if (dx>dy)
      {
        /* x ist schnelle Richtung */
        pdx=incx; pdy=0;    /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        es =dy;   el =dx;   /* Fehlerschritte schnell, langsam */
      } else
      {
        /* y ist schnelle Richtung */
        pdx=0;    pdy=incy; /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        es =dx;   el =dy;   /* Fehlerschritte schnell, langsam */
      }
      /* Initialisierungen vor Schleifenbeginn */
      x = path[start].x();
      y = path[start].y();
      err = el/2;
      /* Pixel berechnen */
      for(t=0; t<el; ++t) /* t zaehlt die Pixel, el ist auch Anzahl */
      {
        /* Aktualisierung Fehlerterm */
        err -= es; 
        if(err<0)
        {
          /* Fehlerterm wieder positiv (>=0) machen */
          err += el;
          /* Schritt in langsame Richtung, Diagonalschritt */
          x += ddx;
          y += ddy;
        } else
        {
          /* Schritt in schnelle Richtung, Parallelschritt */
          x += pdx;
          y += pdy;
        }
      
        // --- start: check if obstacle around
        if ( m_ObstacleTransform->getValue ( x, y ) < m_MinAllowedObstacleDistance )
        {
          ok = false;
          break;
        }
        // --- end  : check if obstacle around
      } // Pixel berechnen
      
      if ( ok )
      {
        maxVal = end;
      }
    } // for: inner
    newPath.push_back( path[maxVal] );
    start = maxVal; // incremented by foor loop to max+1
  } // for: outer
  // END: P2AT HACK

  success = true;
  return newPath;
#endif
}

bool Explorer::getNearestFrontier(Eigen::Vector2i& nextFrontier) {
    if (!m_OccupancyMap) {
        ROS_ERROR("Missing occupancy map. Aborting.");
        return false;
    }

    computeFrontierMap();
    computeDrivingDistanceTransform();

    bool found = false;
    int distXPos = -1;
    int distYPos = -1;
    double dist = 10000000;
    for (int y = 0; y < m_TargetMap->height(); y++) {
        for (int x = 0; x < m_TargetMap->width(); x++) {
            if (m_TargetMap->getValue(x, y) == 1 &&
                m_DrivingDistanceTransform->getValue(x, y) < 999999) {
                if (m_DrivingDistanceTransform->getValue(x, y) < dist) {
                    found = true;
                    dist = m_DrivingDistanceTransform->getValue(x, y);
                    distXPos = x;
                    distYPos = y;
                }
            }
        }
    }
    if (found) {
        nextFrontier.x() = distXPos;
        nextFrontier.y() = distYPos;
        return true;
    } else {
        return false;
    }
}

// HELPERS
// //////////////////////////////////////////////////////////////////////////////////////////////////////////

void Explorer::distanceFloodFill(GridMap<double>* map, Eigen::Vector2i start) {
    if (!map) {
        ROS_ERROR("Received 0-pointer!");
    }

    computeObstacleTransform();

    int width = map->width();
    int height = map->height();
    map->fill(MAX_DISTANCE);

    int fromX = start.x();
    int fromY = start.y();
    map->setValue(fromX, fromY, 0);

    queue<int> xQueue;
    queue<int> yQueue;
    xQueue.push(fromX + 1);
    yQueue.push(fromY);
    xQueue.push(fromX - 1);
    yQueue.push(fromY);
    xQueue.push(fromX);
    yQueue.push(fromY - 1);
    xQueue.push(fromX);
    yQueue.push(fromY + 1);
    int xVal, yVal;
    while (!xQueue.empty()) {
        xVal = xQueue.front();
        yVal = yQueue.front();
        xQueue.pop();
        yQueue.pop();
        bool isFree = (m_OccupancyMap->getValue(xVal, yVal) < UNKNOWN ||
                       m_OccupancyMap->getValue(xVal, yVal) !=
                           NOT_SEEN_YET);  // only fill free cells
        bool isSafe = m_ObstacleTransform->getValue(xVal, yVal) >
                      m_MinAllowedObstacleDistance;
        if (xVal > 0 && xVal < width - 1 && yVal > 0 && yVal < height - 1 &&
            isFree && isSafe) {
            float value = map->getValue(xVal, yVal);
            float value_u = map->getValue(xVal, yVal - 1) + 1;
            float value_d = map->getValue(xVal, yVal + 1) + 1;
            float value_l = map->getValue(xVal - 1, yVal) + 1;
            float value_r = map->getValue(xVal + 1, yVal) + 1;

            float value_ur = map->getValue(xVal + 1, yVal - 1) + 1.4142;
            float value_ul = map->getValue(xVal - 1, yVal - 1) + 1.4142;
            float value_ll = map->getValue(xVal - 1, yVal + 1) + 1.4142;
            float value_lr = map->getValue(xVal + 1, yVal + 1) + 1.4142;

            float min1 = value_u < value_d ? value_u : value_d;
            float min2 = value_l < value_r ? value_l : value_r;
            float min3 = value_ur < value_ul ? value_ur : value_ul;
            float min4 = value_ll < value_lr ? value_ll : value_lr;
            float min12 = min1 < min2 ? min1 : min2;
            float min34 = min3 < min4 ? min3 : min4;
            float min = min12 < min34 ? min12 : min34;
            float newVal = min;
            if (value > newVal) {
                map->setValue(xVal, yVal, newVal);
                if (map->getValue(xVal, yVal + 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal, yVal - 1) > newVal + 1) {
                    xQueue.push(xVal);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal) > newVal + 1) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal - 1, yVal) > newVal + 1) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal);
                }
                if (map->getValue(xVal + 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal - 1, yVal - 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal - 1);
                }
                if (map->getValue(xVal + 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal + 1);
                    yQueue.push(yVal + 1);
                }
                if (map->getValue(xVal - 1, yVal + 1) > newVal + 1.4142) {
                    xQueue.push(xVal - 1);
                    yQueue.push(yVal + 1);
                }
            }
        }
    }
}

// Implementation taken from http://www.cs.cmu.edu/~cil/vnew.html
double* Explorer::distanceTransform1D(double* f, int n) {
    // int width = m_OccupancyMap->width();
    // int height = m_OccupancyMap->height();
    // double maxDistance = height > width ? height : width;

    double* d = new double[n];
    int* v = new int[n];
    double* z = new double[n + 1];
    int k = 0;
    v[0] = 0;
    z[0] = -INT_MAX;
    z[1] = INT_MAX;
    for (int q = 1; q <= n - 1; q++) {
        double s =
            ((f[q] + (q * q)) - (f[v[k]] + (v[k] * v[k]))) / (2 * q - 2 * v[k]);
        while (s <= z[k]) {
            k--;
            s = ((f[q] + (q * q)) - (f[v[k]] + (v[k] * v[k]))) /
                (2 * q - 2 * v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INT_MAX;
    }

    k = 0;
    for (int q = 0; q <= n - 1; q++) {
        while (z[k + 1] < q) k++;
        d[q] = ((q - v[k]) * (q - v[k])) + f[v[k]];
    }

    delete[] v;
    delete[] z;
    return d;
}
