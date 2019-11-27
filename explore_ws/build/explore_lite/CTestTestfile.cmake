# CMake generated Testfile for 
# Source directory: /home/anton/explore_ws/src/explore
# Build directory: /home/anton/explore_ws/build/explore_lite
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_explore_lite_roslaunch-check_launch "/home/anton/explore_ws/build/explore_lite/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/anton/explore_ws/build/explore_lite/test_results/explore_lite/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/anton/explore_ws/build/explore_lite/test_results/explore_lite" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/anton/explore_ws/build/explore_lite/test_results/explore_lite/roslaunch-check_launch.xml' '/home/anton/explore_ws/src/explore/launch' ")
subdirs(gtest)
