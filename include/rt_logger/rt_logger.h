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
