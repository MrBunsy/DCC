cmake_minimum_required (VERSION 2.6)
project (TrainTracker)
find_package( OpenCV REQUIRED )
add_executable(TrainTracker traintracker.cxx)
target_link_libraries( TrainTracker ${OpenCV_LIBS} )