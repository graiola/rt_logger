/*
 * Copyright 2019 Gennaro Raiola
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Gennaro Raiola
 */

#ifndef RT_LOGGER_ROSNODE_H
#define RT_LOGGER_ROSNODE_H

#include <ros/ros.h>

namespace rt_logger
{


class RosNode
{

public:

    /**
     * @brief Shared pointer to RosNode
     */
    typedef std::shared_ptr<RosNode> Ptr;

    /**
     * @brief Shared pointer to const RosNode
     */
    typedef std::shared_ptr<const RosNode> ConstPtr;

    RosNode(std::string ros_node_name)
    {
        init(ros_node_name);
    }

    RosNode()
    {
        init_ = false;
        ros_nh_ptr_ = NULL;
    }

    void init(std::string ros_node_name)
    {
        int argc = 1;
        char* arg0 = strdup(ros_node_name.c_str());
        char* argv[] = {arg0, 0};
        ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
        free (arg0);
        if(ros::master::check())
        {
            ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
        }
        else
        {
            std::string err("roscore not found... did you start the server?");
            throw std::runtime_error(err);
        }
        init_ = true;
    }

    ~RosNode()
    {
        if(ros_nh_ptr_!=NULL && init_ == true)
        {
            ros_nh_ptr_->shutdown();
            delete ros_nh_ptr_;
        }
    }

    ros::NodeHandle& getNode()
    {
        if(init_ == true)
            return *ros_nh_ptr_;
        else
        {
            std::string err("RosNode not initialized");
            throw std::runtime_error(err);
        }
    }

    bool reset()
    {
        if(init_ == true)
        {
            ros_nh_ptr_->shutdown();
            delete ros_nh_ptr_;
            ros_nh_ptr_ = NULL;
            init_ = false;
            return true;
        }
        else
        {
            std::string err("RosNode not initialized");
            throw std::runtime_error(err);
            return false;
        }
    }

    bool initDone()
    {
        return init_;
    }

protected:
    ros::NodeHandle* ros_nh_ptr_;
    bool init_;
};

} // namespace

#endif
