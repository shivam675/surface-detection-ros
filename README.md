# surface-detecction
All the packages are 3rd party. The main project is to enable robots to detect a surface from depth camera and plan a path in such a way that the polishing tool polishes the entire surface




### Cmake version 3.20.3 or higher required.

## Use only catkin build.
build likewise
    `catkin build -DCMAKE_BUILD_TYPE=Release -j$(nproc)`

    
    
For trying out successful build 
 `roslaunch godel_irb2400_support irb2400_blending.launch`
     
    
#### All the major credits goes to godel, abb, descartes, godel_openvoronoi, industrial_core, keyence_experimnetal, libsocket, swri_profiler for main packages.
##### Thankyou Kevin Patel for all the hardwork for making these packages error free for ROS Melodic.

##Tested on:
 $ Ubuntu 18.04
 $ ROS melodic
 $ opencv - build - 4.5.2-dev
