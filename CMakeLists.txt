cmake_minimum_required(VERSION 3.8)
project(monkey_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS
    ament_cmake
    rclcpp
    behaviortree_cpp
    behaviortree_ros2
    action_tutorials_interfaces
    btcpp_ros2_interfaces
    std_srvs
    std_msgs
    tf2_geometry_msgs
    monkey_bt_util
    monkey_std_msgs)

# Find each package in the list
foreach(dep ${THIS_PACKAGE_DEPS})
    find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

# Function to create and configure BT plugins
function(create_bt_plugin plugin_name src_file)
    add_library(${plugin_name} SHARED ${src_file})
    target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
    ament_target_dependencies(${plugin_name} ${THIS_PACKAGE_DEPS})

    # INSTALL plugins for other packages to load
    install(TARGETS 
        ${plugin_name}
        LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
        ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
        RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
    )
endfunction()

# Automatically find and create plugins from specified directories
file(GLOB_RECURSE ACTION_PLUGIN_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "plugins/btros2_actions/*.cpp")
file(GLOB_RECURSE SERVICE_PLUGIN_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "plugins/btros2_services/*.cpp")

# Create plugins using the discovered source files
foreach(src_file ${ACTION_PLUGIN_SOURCES})
    get_filename_component(plugin_name ${src_file} NAME_WE)
    create_bt_plugin(${plugin_name} ${src_file})
endforeach()

foreach(src_file ${SERVICE_PLUGIN_SOURCES})
    get_filename_component(plugin_name ${src_file} NAME_WE)
    create_bt_plugin(${plugin_name} ${src_file})
endforeach()

# --------Install---------
install(DIRECTORY include/
    DESTINATION include/
)

install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()
