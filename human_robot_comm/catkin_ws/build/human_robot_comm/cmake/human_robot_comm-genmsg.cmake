# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "human_robot_comm: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ihuman_robot_comm:/home/jerry/catkin_ws/src/human_robot_comm/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(human_robot_comm_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(human_robot_comm
  "/home/jerry/catkin_ws/src/human_robot_comm/msg/floatlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_robot_comm
)

### Generating Services

### Generating Module File
_generate_module_cpp(human_robot_comm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_robot_comm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(human_robot_comm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(human_robot_comm_generate_messages human_robot_comm_generate_messages_cpp)

# target for backward compatibility
add_custom_target(human_robot_comm_gencpp)
add_dependencies(human_robot_comm_gencpp human_robot_comm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_robot_comm_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(human_robot_comm
  "/home/jerry/catkin_ws/src/human_robot_comm/msg/floatlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_robot_comm
)

### Generating Services

### Generating Module File
_generate_module_lisp(human_robot_comm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_robot_comm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(human_robot_comm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(human_robot_comm_generate_messages human_robot_comm_generate_messages_lisp)

# target for backward compatibility
add_custom_target(human_robot_comm_genlisp)
add_dependencies(human_robot_comm_genlisp human_robot_comm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_robot_comm_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(human_robot_comm
  "/home/jerry/catkin_ws/src/human_robot_comm/msg/floatlist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_robot_comm
)

### Generating Services

### Generating Module File
_generate_module_py(human_robot_comm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_robot_comm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(human_robot_comm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(human_robot_comm_generate_messages human_robot_comm_generate_messages_py)

# target for backward compatibility
add_custom_target(human_robot_comm_genpy)
add_dependencies(human_robot_comm_genpy human_robot_comm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS human_robot_comm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_robot_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/human_robot_comm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(human_robot_comm_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_robot_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/human_robot_comm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(human_robot_comm_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_robot_comm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_robot_comm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/human_robot_comm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(human_robot_comm_generate_messages_py std_msgs_generate_messages_py)
