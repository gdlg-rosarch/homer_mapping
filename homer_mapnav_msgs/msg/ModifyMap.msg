int32 HIGH_SENSITIV = -2  # Area is highly sensitiv for Obstacles
int32 NOT_MASKED = -1     # Area is not masked: use slam map
int32 FREE = 0            # Area is free
int32 OBSTACLE = 99       # Area is occupied by an obstacle (e.g. 3d scan) 
int32 BLOCKED = 100       # Area blocked by user (use UNBLOCK to .. )
int32 HOKUYO = 101		  # Area is blocked by Hokuyo Obstacle
int32 KINECT = 102 		  # Area is blocked by Kinect Obstacle
int32 DEPTH = 102		  # Area is blocked by Depth Camera Obstacle
int32 RAPID = 103		  # Area is blocked by Rapid Mapping



int32 SET_FREE = 0      	  # Mark area free, area must not be BLOCKED
int32 SET_BLOCKED = 100       # Block area
int32 SET_OBSTACLE = 99   	  # Mark area as occupied by an obstacle (e.g. 3d scan)
int32 SET_HIGH_SENSITIV = -2  # Mark Area as highly sensitiv for Obstacles


geometry_msgs/Point[] region
int32 maskAction
int32 mapLayer #currently not used. modifications only take place in the masking map layer
