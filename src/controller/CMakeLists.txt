set(EXE_TARGET_NAME tactile-magnetic-sensor-controller)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             math
             sig
             )

# ICUB
find_package(ICUB REQUIRED)

set(${EXE_TARGET_NAME}_HDR
    include/Controller.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Controller.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/tactile_magnetic_sensor_controller.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
               )

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      YARP::YARP_OS
                      YARP::YARP_init
                      YARP::YARP_math
                      YARP::YARP_sig
                      ${ICUB_LIBRARIES}
                      )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini)

file(GLOB ${EXE_TARGET_NAME}_XML ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/tactile-magnetic-sensor-controller)
yarp_install(FILES ${${EXE_TARGET_NAME}_XML}  DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
