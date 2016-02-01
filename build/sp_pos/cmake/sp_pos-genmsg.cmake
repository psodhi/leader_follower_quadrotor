# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sp_pos: 2 messages, 0 services")

set(MSG_I_FLAGS "-Isp_pos:/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sp_pos_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/utmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_pos
)
_generate_msg_cpp(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/latlonData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_pos
)

### Generating Services

### Generating Module File
_generate_module_cpp(sp_pos
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_pos
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sp_pos_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sp_pos_generate_messages sp_pos_generate_messages_cpp)

# target for backward compatibility
add_custom_target(sp_pos_gencpp)
add_dependencies(sp_pos_gencpp sp_pos_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_pos_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/utmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_pos
)
_generate_msg_lisp(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/latlonData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_pos
)

### Generating Services

### Generating Module File
_generate_module_lisp(sp_pos
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_pos
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sp_pos_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sp_pos_generate_messages sp_pos_generate_messages_lisp)

# target for backward compatibility
add_custom_target(sp_pos_genlisp)
add_dependencies(sp_pos_genlisp sp_pos_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_pos_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/utmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos
)
_generate_msg_py(sp_pos
  "/home/paloma/catkin_ws/mavros_ws/src/sp_pos/msg/latlonData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos
)

### Generating Services

### Generating Module File
_generate_module_py(sp_pos
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sp_pos_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sp_pos_generate_messages sp_pos_generate_messages_py)

# target for backward compatibility
add_custom_target(sp_pos_genpy)
add_dependencies(sp_pos_genpy sp_pos_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_pos_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_pos
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sp_pos_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_pos
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sp_pos_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_pos
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sp_pos_generate_messages_py std_msgs_generate_messages_py)
