# CMake generated Testfile for 
# Source directory: /home/andre/ros2_ws/src/apriltag_ros-master
# Build directory: /home/andre/ros2_ws/build/apriltag_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(clang_format "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/andre/ros2_ws/build/apriltag_ros/test_results/apriltag_ros/clang_format.xunit.xml" "--package-name" "apriltag_ros" "--output-file" "/home/andre/ros2_ws/build/apriltag_ros/ament_clang_format/clang_format.txt" "--command" "/opt/ros/humble/bin/ament_clang_format" "--xunit-file" "/home/andre/ros2_ws/build/apriltag_ros/test_results/apriltag_ros/clang_format.xunit.xml" "--config" "/home/andre/ros2_ws/src/apriltag_ros-master/.clang-format")
set_tests_properties(clang_format PROPERTIES  LABELS "clang_format;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/andre/ros2_ws/src/apriltag_ros-master" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_clang_format/cmake/ament_clang_format.cmake;57;ament_add_test;/opt/ros/humble/share/ament_cmake_clang_format/cmake/ament_cmake_clang_format_lint_hook.cmake;27;ament_clang_format;/opt/ros/humble/share/ament_cmake_clang_format/cmake/ament_cmake_clang_format_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/andre/ros2_ws/src/apriltag_ros-master/CMakeLists.txt;90;ament_package;/home/andre/ros2_ws/src/apriltag_ros-master/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/andre/ros2_ws/build/apriltag_ros/test_results/apriltag_ros/cppcheck.xunit.xml" "--package-name" "apriltag_ros" "--output-file" "/home/andre/ros2_ws/build/apriltag_ros/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/andre/ros2_ws/build/apriltag_ros/test_results/apriltag_ros/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/andre/ros2_ws/src/apriltag_ros-master" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/andre/ros2_ws/src/apriltag_ros-master/CMakeLists.txt;90;ament_package;/home/andre/ros2_ws/src/apriltag_ros-master/CMakeLists.txt;0;")
