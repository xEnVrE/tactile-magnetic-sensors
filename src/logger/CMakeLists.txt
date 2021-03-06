set(EXE_TARGET_NAME tactile-magnetic-sensor-logger)

# Eigen
find_package(Eigen3 QUIET CONFIG)
if(NOT EIGEN3_FOUND)
    # in case the cmake/FindEigen3.cmake fails
    find_package(Eigen3 REQUIRED)
endif()

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             cv
             )

# ICUB
find_package(ICUB REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Bayes Filters
find_package(BayesFilters 0.9.0 QUIET)
if(NOT BayesFilters_FOUND)
    message(STATUS "Did not found required master release of BayesFilters. Looking for devel version.")
    find_package(BayesFilters 0.9.100 REQUIRED)
endif()

set(${EXE_TARGET_NAME}_HDR
    include/Logger.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Logger.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/logger.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      BayesFilters::BayesFilters
                      YARP::YARP_init
                      YARP::YARP_OS
                      YARP::YARP_sig
                      YARP::YARP_eigen
                      YARP::YARP_cv
                      iCubFingersEncoders
                      ${ICUB_LIBRARIES}
                      ${OpenCV_LIBS}
                      )

if(NOT TARGET Eigen3)
    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               ${EIGEN3_INCLUDE_DIR}
                               )
else()
    target_link_libraries(${EXE_TARGET_NAME}
                          PRIVATE
                          Eigen3::Eigen
                          )
endif()

set(${EXE_TARGET_NAME}_CONF
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini
    ${CMAKE_CURRENT_SOURCE_DIR}/conf/analogs_configuration.ini
    )

file(GLOB ${EXE_TARGET_NAME}_XML ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/tactile-magnetic-sensor-logger)
yarp_install(FILES ${${EXE_TARGET_NAME}_XML}  DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
