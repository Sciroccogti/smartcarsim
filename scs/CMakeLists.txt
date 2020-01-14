cmake_minimum_required(VERSION 3.5)
project(scs)

include_directories(include)

add_library(${PROJECT_NAME}
  src/api.cpp
  src/car.cpp
  src/draw.cpp
  src/simulation.cpp
  src/track.cpp
  src/ui.cpp
  src/vector.cpp
)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
