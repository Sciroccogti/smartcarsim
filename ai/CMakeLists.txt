cmake_minimum_required(VERSION 3.5)
project(scs_ai)
message(STATUS ${CMAKE_INSTALL_PREFIX})

include_directories(include)
link_directories(${CMAKE_INSTALL_PREFIX}/scs)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} scs glut GLU GL ode)

install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME})
