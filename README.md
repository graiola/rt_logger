# rt_logger

A simple ROS package to log numeric data such as doubles, std vectors, Eigen matrices and vectors.

## How to use

Include this header wherever you want to access the logger:

`#include <rt_logger/rt_logger.h>`

Add a real time publisher to the logger:

` RtLogger::getLogger().addPublisher("/topic_name",data_to_publish) `

*<h6>Note: This operation is not real time safe! If you are using a real time controller from ros-control, perform this operation in the init of the controller.</h6>*

Finally, add this line in the main loop of your code to publish the data:

` RtLogger::getLogger().publish(ros_time)`

Check out this [test](https://github.com/graiola/rt_logger/blob/master/test/rt_logger_test.cpp) for more examples.

