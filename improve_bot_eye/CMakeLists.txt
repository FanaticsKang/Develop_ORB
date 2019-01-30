cmake_minimum_required (VERSION 2.8.11)
project(Orb)

# Include the header file so cmake can find it when compiling under
# other projects that depend on Orb
add_library(${PROJECT_NAME} SHARED
 ORBextractor.h
 ORBextractor.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${USER_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
 ${OpenCV_LIBRARIES}
 ${GFLAGS_LIBRARIES}
 ${GLOG_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION XP/lib_${CMAKE_SYSTEM_PROCESSOR}
  RUNTIME DESTINATION XP/lib_${CMAKE_SYSTEM_PROCESSOR}  # for cygwinx
)

# post build for cygwin
# Copy the generated library (.dll) to /usr/local/bin
if (CYGWIN)
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      $<TARGET_FILE:${PROJECT_NAME}>
      /usr/local/bin/$<TARGET_FILE_NAME:${PROJECT_NAME}>
  )
endif()
