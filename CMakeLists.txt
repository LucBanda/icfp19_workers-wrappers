cmake_minimum_required(VERSION 2.6)
PROJECT (icfp19)

include_directories (./src ./fuqinho . /usr/local/include ./openGA/src)
link_directories(/usr/lib /usr/bin /usr/local/lib)

add_definitions(-MD -MP -Wall -O3 -std=c++11 -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (viz_ww src/main.cpp src/mine.cpp src/renderer.cpp src/agent.cpp src/fileparser.cpp)
add_executable (solver src/renderer.cpp src/solver.cpp src/genetic_splitter.cpp src/genetic_orderer.cpp src/genetic_optimizer.cpp src/mine.cpp src/agent.cpp src/fileparser.cpp)

target_link_libraries(solver libemon.a allegro allegro_primitives allegro_main allegro_font allegro_ttf pthread)
target_link_libraries(viz_ww allegro allegro_primitives allegro_main allegro_font allegro_ttf libemon.a)
