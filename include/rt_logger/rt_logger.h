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

#ifndef RT_LOGGER_RT_LOGGER_H
#define RT_LOGGER_RT_LOGGER_H


#ifdef ROS
#include <ros/ros.h>
#include <rt_logger/ros/publishers.h>

namespace rt_logger
{

class RtLogger
{
public:

    static RtLogger& getLogger()
    {
        static RtLogger logger;
        return logger;
    }

    template <typename data_t>
    void addPublisher(const std::string& topic_name, const data_t& data, const std::string& data_name = "")
    {
        publishers_->addPublisher(topic_name,&data,data_name);
    }

    void publish(const ros::Time& time)
    {
        publishers_->publish(time);
    }

    void publish(const ros::Time& time, const std::string& topic_name)
    {
        publishers_->publish(time,topic_name);
    }

    void removePublisher(const std::string& topic_name)
    {
        publishers_->removePublisher(topic_name);
    }

    void removePublishers()
    {
        publishers_->removePublishers();
    }

private:

  RtLogger()
  {
      publishers_.reset(new PublishersManager(nh_));
  }

  //~RtLogger()

  RtLogger(const RtLogger&)= delete;
  RtLogger& operator=(const RtLogger&)= delete;

  PublishersManager::Ptr publishers_;
  ros::NodeHandle nh_;

};

} // namespace

#elif defined(ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rt_logger/ros2/publishers.h>

namespace rt_logger
{

class RtLogger
{
public:
    static RtLogger& getLogger()
    {
        static RtLogger logger;
        return logger;
    }

    template <typename data_t>
    void addPublisher(const std::string& topic_name, const data_t& data, const std::string& data_name = "")
    {
        publishers_->addPublisher(topic_name, &data, data_name);
    }

    void publish(const rclcpp::Time& time)
    {
        publishers_->publish(time);
    }

    void publish(const rclcpp::Time& time, const std::string& topic_name)
    {
        publishers_->publish(time, topic_name);
    }

    void removePublisher(const std::string& topic_name)
    {
        publishers_->removePublisher(topic_name);
    }

    void removePublishers()
    {
        publishers_->removePublishers();
    }

private:
    RtLogger() : node_(nullptr)
    {
        node_ = std::make_shared<rclcpp::Node>("rt_logger_node");
        publishers_ = std::make_shared<PublishersManager>(node_);
    }

    // Prevent copying and assignment
    RtLogger(const RtLogger&) = delete;
    RtLogger& operator=(const RtLogger&) = delete;

    PublishersManager::Ptr publishers_;
    std::shared_ptr<rclcpp::Node> node_;
};

} // namespace

#endif

#endif
