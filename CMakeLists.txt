cmake_minimum_required(VERSION 3.0.2)
project(iiwapy)

find_package(catkin)

catkin_python_setup()

catkin_package()

catkin_add_nosetests(tests)
