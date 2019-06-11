cmake_minimum_required(VERSION 2.8.3)
project(APFinger)

# force_proximity_ros can not be found on build.ros.org
# find_package(force_proximity_ros QUIET)
# if(force_proximity_ros_FOUND)
#   set(FORCE_PROXIMITY_ROS_PKG force_proximity_ros)
# else()
#   message(WARNING "COULD NOT BE FOUND force_proximity_ros PACKAGE")
# endif()

find_package(catkin REQUIRED COMPONENTS
    ${FORCE_PROXIMITY_ROS_PKG}
    message_generation
    roseus
    std_msgs
    # shape_msgs
    # jsk_recognition_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES
    DistanceAndProximity.msg
    GripperPosAndProximity.msg
    ProximityStates.msg
    Proximities.msg
    # PlaneProximities.msg
    ProximityDistances.msg
    Proximity.msg
    ProximityArray.msg
    ProximityStamped.msg
)

# add_service_files(
#     FILES
# )

# add_action_files(
#     DIRECTORY action
# )

generate_messages(
    DEPENDENCIES
    std_msgs
    # jsk_recognition_msgs
    # shape_msgs
    # ${FORCE_PROXIMITY_ROS_PKG}
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# XXX

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    # ${FORCE_PROXIMITY_ROS_PKG}
    std_msgs
    # jsk_recognition_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# find_package(PkgConfig)

## Declare a C++ executable
# add_executable()

## Add cmake target dependencies of the executable
# add_dependencies()

## Specify libraries to link a library or executable target against
# target_link_libraries()

#############
## Install ##
#############

# XXX

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
# endif()
