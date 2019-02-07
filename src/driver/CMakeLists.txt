project(tactile-magnetic-sensor-driver)

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(${LIBRARY_TARGET_NAME}_HDR include/skinSensor.h
                               include/socketcan.h
                               include/taxel.h
                               )

set(${LIBRARY_TARGET_NAME}_SRC src/skinSensor.cpp
                               src/socketcan.cpp
                               src/taxel.cpp
                               )

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_HDR} ${${LIBRARY_TARGET_NAME}_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} Threads::Threads)
