project(frame_utils)

add_library(${PROJECT_NAME} SHARED
    ./data_align.cc
    ./camera_model.cc
    )

target_link_libraries(${PROJECT_NAME} input_source)
