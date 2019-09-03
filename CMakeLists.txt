cmake_minimum_required(VERSION 2.6)
PROJECT (icfp19)

include_directories (./src ./fuqinho ./fuqinho/glog/src ./fuqinho/glog/src/glog ./fuqinho/glog/_virtual_includes/glog/ . /usr/local/include ./openGA/src)
link_directories(/usr/lib /usr/bin /usr/local/lib /Users/luc/icfp19_workers-wrappers/fuqinho/gflags /Users/luc/icfp19_workers-wrappers/fuqinho/glog)

add_definitions(-MD -MP -Wall -march=native -O3 -std=c++11 -Wshadow -Woverloaded-virtual -DNOSSL -fPIC)
add_executable (viz_ww src/main.cpp src/mine.cpp src/renderer.cpp src/agent.cpp src/fileparser.cpp)
add_executable (optimizer src/mine.cpp src/agent.cpp src/genetic_optimizer.cpp)
add_executable (solver src/mine.cpp src/agent.cpp src/solver.cpp)
add_executable (fuq-solver fuqinho/main.cc)

target_link_libraries(solver libemon.a)
target_link_libraries(fuq-solver libgflags.a libglog.a)
target_link_libraries(optimizer libemon.a)
target_link_libraries(viz_ww allegro allegro_primitives allegro_main allegro_font allegro_ttf libemon.a)


