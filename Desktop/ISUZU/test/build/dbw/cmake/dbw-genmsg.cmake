# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dbw: 5 messages, 0 services")

set(MSG_I_FLAGS "-Idbw:/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dbw_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_custom_target(_dbw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dbw" "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Header"
)

get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_custom_target(_dbw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dbw" "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" ""
)

get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_custom_target(_dbw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dbw" "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" "dbw/GPSStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_custom_target(_dbw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dbw" "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_custom_target(_dbw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dbw" "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
)
_generate_msg_cpp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
)
_generate_msg_cpp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
)
_generate_msg_cpp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
)
_generate_msg_cpp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
)

### Generating Services

### Generating Module File
_generate_module_cpp(dbw
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dbw_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dbw_generate_messages dbw_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_dependencies(dbw_generate_messages_cpp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_dependencies(dbw_generate_messages_cpp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_dependencies(dbw_generate_messages_cpp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_dependencies(dbw_generate_messages_cpp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_dependencies(dbw_generate_messages_cpp _dbw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dbw_gencpp)
add_dependencies(dbw_gencpp dbw_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dbw_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
)
_generate_msg_eus(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
)
_generate_msg_eus(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
)
_generate_msg_eus(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
)
_generate_msg_eus(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
)

### Generating Services

### Generating Module File
_generate_module_eus(dbw
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dbw_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dbw_generate_messages dbw_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_dependencies(dbw_generate_messages_eus _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_dependencies(dbw_generate_messages_eus _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_dependencies(dbw_generate_messages_eus _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_dependencies(dbw_generate_messages_eus _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_dependencies(dbw_generate_messages_eus _dbw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dbw_geneus)
add_dependencies(dbw_geneus dbw_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dbw_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
)
_generate_msg_lisp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
)
_generate_msg_lisp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
)
_generate_msg_lisp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
)
_generate_msg_lisp(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
)

### Generating Services

### Generating Module File
_generate_module_lisp(dbw
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dbw_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dbw_generate_messages dbw_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_dependencies(dbw_generate_messages_lisp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_dependencies(dbw_generate_messages_lisp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_dependencies(dbw_generate_messages_lisp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_dependencies(dbw_generate_messages_lisp _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_dependencies(dbw_generate_messages_lisp _dbw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dbw_genlisp)
add_dependencies(dbw_genlisp dbw_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dbw_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
)
_generate_msg_nodejs(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
)
_generate_msg_nodejs(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
)
_generate_msg_nodejs(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
)
_generate_msg_nodejs(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dbw
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dbw_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dbw_generate_messages dbw_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_dependencies(dbw_generate_messages_nodejs _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_dependencies(dbw_generate_messages_nodejs _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_dependencies(dbw_generate_messages_nodejs _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_dependencies(dbw_generate_messages_nodejs _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_dependencies(dbw_generate_messages_nodejs _dbw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dbw_gennodejs)
add_dependencies(dbw_gennodejs dbw_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dbw_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
)
_generate_msg_py(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
)
_generate_msg_py(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
)
_generate_msg_py(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg"
  "${MSG_I_FLAGS}"
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
)
_generate_msg_py(dbw
  "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
)

### Generating Services

### Generating Module File
_generate_module_py(dbw
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dbw_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dbw_generate_messages dbw_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/FloatArray.msg" NAME_WE)
add_dependencies(dbw_generate_messages_py _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/SteerCmd.msg" NAME_WE)
add_dependencies(dbw_generate_messages_py _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSFix.msg" NAME_WE)
add_dependencies(dbw_generate_messages_py _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/GPSStatus.msg" NAME_WE)
add_dependencies(dbw_generate_messages_py _dbw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qiyang-isuzu/Desktop/ISUZU/test/src/dbw/msg/Num.msg" NAME_WE)
add_dependencies(dbw_generate_messages_py _dbw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dbw_genpy)
add_dependencies(dbw_genpy dbw_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dbw_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dbw
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dbw_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(dbw_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dbw
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dbw_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(dbw_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dbw
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dbw_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(dbw_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dbw
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dbw_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(dbw_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dbw
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dbw_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(dbw_generate_messages_py sensor_msgs_generate_messages_py)
endif()
