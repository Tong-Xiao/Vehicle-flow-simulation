安装依赖
=============================================
> ## sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
> ## sudo apt-get install ros-kinetic-gazebo-ros-control
> ## sudo apt-get install ros-kinetic-velodyne-pointcloud
## 16.04安装sicktoolbox
> ### cd ~ 
> ### mkdir -p sicktoolbox_ws/src
> ### cd sicktoolbox_ws
> ### catkin_make
> ### cd src
> ### git clone https://github.com/SantoshBanisetty/sicktoolbox.git
> ### git clone git@github.com:ros-drivers/sicktoolbox_wrapper.git
> ### cd ..
> ### catkin_make
> ### echo "source ~/sicktoolbox_ws/devel/setup.bash" >> ~/.bashrc
> ### source ~/.bashrc

编译报warning和error
===============================================
> ## sudo gedit ~/multi_car_test/src/CMakeLists.txt
> ## 在文件中的第2或者3或者4或者5行的位置添加：  SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
--------------------
--------------------

编译运行
===============================================
> ## cd multi_car_test
> ## catkin_make
> ## source devel/setup.bash
> ## roslaunch multi_car multi_car.launch

添加车辆
===============================================
参考代码中
----< add car example step 1-4>----部分