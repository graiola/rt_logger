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

#include <ros/ros.h>
#include <rt_logger/publishers.h>

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
    void addPublisher(const std::string& name, const data_t& x)
    {
        publishers_->addPublisher(name,&x);
    }

    void publish(const ros::Time& /*time*/)
    {
        publishers_->publishAll();
    }

private:

  RtLogger()
  {
      ros::NodeHandle logger_nh("rt_logger");
      publishers_.reset(new RealTimePublishers(logger_nh));
  }

  //~RtLogger()

  RtLogger(const RtLogger&)= delete;
  RtLogger& operator=(const RtLogger&)= delete;

  RealTimePublishers::Ptr publishers_;

};


} // namespace


#endif
