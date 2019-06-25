cmake_minimum_required(VERSION 2.8.3)
project(driver_station)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/driver_station
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resources/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)
