# rt_logger

A simple ROS package to log numeric data such as doubles, std vectors, Eigen matrices and vectors.

## How to use

Include this header wherever you want to access the logger:

`#include <rt_logger/rt_logger.h>`

Add a real time publisher to the logger:

` rt_logger::RtLogger::getLogger().addPublisher("/topic_name",data_to_publish) `

*<h6>Note: This operation is not real time safe! If you are using a real time controller from ros-control, perform this operation in the init of the controller.</h6>*

Finally, add this line in the main loop of your code to publish the data:

` rt_logger::RtLogger::getLogger().publish(ros_time)`

Check out this [test](https://github.com/graiola/rt_logger/blob/master/test/rt_logger_test.cpp) for more examples.

## Import data in Matlab or Octave
First, be sure to have [scipy](https://www.scipy.org/) and [numpy](https://numpy.org/) installed:

`sudo apt-get update && sudo apt-get install python-scipy python-numpy -y`

Create the publishers with rt_logger and record the topics with rosbag, see the command line [here](http://wiki.ros.org/rosbag/Commandline)

Use this [script](https://github.com/graiola/rt_logger/blob/master/scripts/read_bag.py) to extract the .mat files from the bag files:

` ./read_bag.py -b test.bag `

This command will create a mat file for each topic in the bag. To see the available options type:

` ./read_bag.py -h `
