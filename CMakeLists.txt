cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(vccs_supervoxel)
find_package(PCL 1.12 REQUIRED COMPONENTS common io segmentation visualization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(APP_HEADERS
        )

SET(APP_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
        )

SET(TARGET_NAME supervoxel)
add_executable(${TARGET_NAME} ${APP_SOURCES} ${APP_HEADERS} )
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 17)
set_target_properties(${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --verbose")

find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(MAIN_LIBS
    ${PCL_LIBRARIES}
)

target_link_libraries(${TARGET_NAME} PRIVATE ${MAIN_LIBS})

target_include_directories(${TARGET_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        )

# PYTHON BINDINGS
# SET(LIB_NAME supervoxel)
# 
# find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(pybind11 REQUIRED)
# 
# set(PYTHON_HOME ${Python3_STDLIB})
# set(ENV{LD_LIBRARY_PATH} "${ENV_LD_LIBRARY_PATH_ORG}")
# 
# SET(PYBIND_SOURCES
#         "${CMAKE_CURRENT_SOURCE_DIR}/src/python_interface.cpp")
# 
# pybind11_add_module(${LIB_NAME} ${APP_SOURCES} ${APP_HEADERS} ${PYBIND_SOURCES})
# target_link_libraries(${LIB_NAME} PUBLIC ${MAIN_LIBS})
# target_include_directories(${LIB_NAME} PUBLIC "include")
# set_target_properties(${LIB_NAME} PROPERTIES CXX_STANDARD 17)
# # 
# # # create in correct destination for inplace use within the build directory
# set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/python3.8/lib-dynload)
# # 
# set(ENV{LD_LIBRARY_PATH} "${ENV_LD_LIBRARY_PATH_ORG}")
# set(PYTHON_HOME ${Python3_STDLIB})
