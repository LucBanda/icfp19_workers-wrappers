cmake_minimum_required(VERSION 2.6)
PROJECT (icfp19)

include_directories (./src ./fuqinho . /usr/local/include ./openGA/src)
link_directories(/usr/lib /usr/bin /usr/local/lib)

add_definitions(-MD -MP -Wall -O3 -std=c++11 -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (viz_ww src/main.cpp src/mine.cpp src/renderer.cpp src/agent.cpp src/fileparser.cpp)
add_executable (optimizer src/mine.cpp src/agent.cpp src/genetic_optimizer.cpp)
add_executable (solver src/mine.cpp src/agent.cpp src/genetic_splitter.cpp src/renderer.cpp)
add_executable (fuq-solver1 fuqinho/main.cc)
add_executable (fuq-solver2 fuqinho/main2.cc)

target_link_libraries(solver libemon.a allegro allegro_primitives allegro_main allegro_font allegro_ttf)
#target_link_libraries(fuq-solver1)
target_link_libraries(optimizer libemon.a)
target_link_libraries(viz_ww allegro allegro_primitives allegro_main allegro_font allegro_ttf libemon.a)
