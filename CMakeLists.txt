project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/Twiddle.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.19.1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(pid2 ${sources})

target_link_libraries(pid2 z ssl uv uWS)
