add_library(object_manager SHARED f_object_manager.cpp)

target_include_directories(object_manager PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS object_manager DESTINATION lib)
