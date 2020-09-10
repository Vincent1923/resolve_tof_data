project(input_source)

add_library(${PROJECT_NAME} SHARED
    ./local_pose_reader.cc
    ./local_depth_reader.cc
    ./local_config.cc
    ./local_type.cc
    )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
