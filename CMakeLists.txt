##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(coordination_oru_ros)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)

find_package(catkin REQUIRED genjava)

find_package(catkin REQUIRED orunav_msgs)

generate_rosjava_messages(
  PACKAGES
    orunav_msgs
)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publish deleteHorribleScripts)

message(WARNING "FROM: ${CATKIN_DEVEL_PREFIX}/../../devel/share/maven")
message(WARNING "TO: ${CATKIN_DEVEL_PREFIX}/share/maven")

#if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/share/maven")
if("${CATKIN_DEVEL_PREFIX}$" MATCHES "isolated")
  message(WARNING "${CATKIN_DEVEL_PREFIX}/share/maven DOES NOT EXIST - will create symlink!!!")
  ADD_CUSTOM_TARGET(link_target ALL 
        COMMAND mkdir -p "${CATKIN_DEVEL_PREFIX}/../../devel/share/maven"
	COMMAND ${CMAKE_COMMAND} -E create_symlink "${CATKIN_DEVEL_PREFIX}/../../devel/share/maven" "${CATKIN_DEVEL_PREFIX}/share/maven"
  )
  ADD_DEPENDENCIES(coordination_oru_ros_generate_artifacts link_target)
else()
  message(WARNING "${CATKIN_DEVEL_PREFIX}/share/maven EXISTS - will not create symlink!!!")
endif()

catkin_package()

##############################################################################
# Installation
##############################################################################

message(WARNING "The devel is: ${CATKIN_DEVEL_PREFIX}")
message(WARNING "The MVN dest is: ${CATKIN_GLOBAL_MAVEN_DESTINATION}")

# Change this to match the maven group name you have specified in the
# allprojects closure the root build.gradle

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/.
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION})
