add_library (ocv 
  opencv_inter.hpp
  opencv_inter.cpp)
add_library (pcloud1 
  pcloud_collector.hpp
  pcloud_collector.cpp)
add_library (pcloud2
  pcd_creator.hpp 
  pcloud_collector2.hpp
  pcloud_collector2.cpp)
add_library (pcdcreator 
  pcd_creator.hpp
  pcd_creator.cpp)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++11 -std=gnu++11> $<$<COMPILE_LANGUAGE:C>:-std=c99>)

install (TARGETS ocv DESTINATION bin)
install (TARGETS pcloud1 DESTINATION bin)
install (TARGETS pcloud2 DESTINATION bin)
install (TARGETS pcdcreator DESTINATION bin)
install (FILES *.hpp DESTINATION include)
