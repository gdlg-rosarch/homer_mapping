cmake_minimum_required(VERSION 2.8.3)

project(homer_mapnav_msgs)

find_package(catkin REQUIRED 
	COMPONENTS message_generation nav_msgs geometry_msgs actionlib_msgs)

add_message_files(FILES
	MapLayers.msg
	ModifyMap.msg
	PointOfInterest.msg
	PointsOfInterest.msg
	RegionOfInterest.msg
	RegionsOfInterest.msg
	ModifyPOI.msg
	TargetUnreachable.msg
	StartNavigation.msg
	NavigateToPOI.msg
	RoiChange.msg
)

add_service_files(FILES
	GetPointsOfInterest.srv
	GetRegionsOfInterest.srv
	GetRegionOfInterestName.srv
	PointInsideRois.srv
	LoadMap.srv
	SaveMap.srv
)

add_action_files(DIRECTORY action FILES
	DetectObstacle.action
	DriveTo.action
)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs)
