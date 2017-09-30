# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lilac_fundamentals: 5 messages, 2 services")

set(MSG_I_FLAGS "-Ililac_fundamentals:/home/liliac/lilac/src/lilac_fundamentals/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ililac_fundamentals:/home/liliac/lilac/src/lilac_fundamentals/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lilac_fundamentals_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg" ""
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg" "lilac_fundamentals/Cell"
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg" ""
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv" ""
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg" "lilac_fundamentals/Cell:lilac_fundamentals/Row"
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv" ""
)

get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg" NAME_WE)
add_custom_target(_lilac_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lilac_fundamentals" "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg;/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)

### Generating Services
_generate_srv_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)
_generate_srv_cpp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
)

### Generating Module File
_generate_module_cpp(lilac_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lilac_fundamentals_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lilac_fundamentals_generate_messages lilac_fundamentals_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_cpp _lilac_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lilac_fundamentals_gencpp)
add_dependencies(lilac_fundamentals_gencpp lilac_fundamentals_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lilac_fundamentals_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg;/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)

### Generating Services
_generate_srv_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)
_generate_srv_lisp(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
)

### Generating Module File
_generate_module_lisp(lilac_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lilac_fundamentals_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lilac_fundamentals_generate_messages lilac_fundamentals_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_lisp _lilac_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lilac_fundamentals_genlisp)
add_dependencies(lilac_fundamentals_genlisp lilac_fundamentals_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lilac_fundamentals_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)
_generate_msg_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg"
  "${MSG_I_FLAGS}"
  "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg;/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)

### Generating Services
_generate_srv_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)
_generate_srv_py(lilac_fundamentals
  "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
)

### Generating Module File
_generate_module_py(lilac_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lilac_fundamentals_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lilac_fundamentals_generate_messages lilac_fundamentals_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Pose.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Row.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/ActualLocalization.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/ExecutePlan.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Grid.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/srv/MoveToPosition.srv" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liliac/lilac/src/lilac_fundamentals/msg/Cell.msg" NAME_WE)
add_dependencies(lilac_fundamentals_generate_messages_py _lilac_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lilac_fundamentals_genpy)
add_dependencies(lilac_fundamentals_genpy lilac_fundamentals_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lilac_fundamentals_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lilac_fundamentals
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(lilac_fundamentals_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(lilac_fundamentals_generate_messages_cpp lilac_fundamentals_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lilac_fundamentals
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(lilac_fundamentals_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(lilac_fundamentals_generate_messages_lisp lilac_fundamentals_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lilac_fundamentals
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(lilac_fundamentals_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(lilac_fundamentals_generate_messages_py lilac_fundamentals_generate_messages_py)
