cmake_minimum_required(VERSION 2.6)
PROJECT (icfp19)

include_directories (./src /usr/local/include ./openGA/src)
link_directories(/usr/lib /usr/bin /usr/local/lib)

add_definitions(-DALLEGRO -MD -MP -Wall -Wsign-compare -O0 -std=c++11 -Wshadow -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (viz_ww src/main.cpp src/mine.cpp src/renderer.cpp src/agent.cpp src/fileparser.cpp)
add_executable (optimizer src/mine.cpp src/agent.cpp src/genetic_optimizer.cpp)

target_link_libraries(viz_ww allegro allegro_primitives allegro_main allegro_font allegro_ttf)
target_link_libraries(optimizer allegro allegro_main)