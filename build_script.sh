#!/bin/bash

catkin build --catkin-make-args tests
catkin build --catkin-make-args run_tests
catkin_test_results ./build_isolated
