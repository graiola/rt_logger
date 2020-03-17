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

#ifndef RT_LOGGER_PUBLISHERS_H
#define RT_LOGGER_PUBLISHERS_H

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <rt_logger/LoggerNumeric.h>
#include <eigen3/Eigen/Core>
#include <type_traits>

namespace rt_logger
{

class RealTimePublisherInterface
{
public:

    /** Initialize the real time publisher. */
    RealTimePublisherInterface(){}

    virtual ~RealTimePublisherInterface(){}

    /** Publish the topic. */
    virtual void publish(const ros::Time& time) = 0;

    inline std::string getTopic(){return topic_name_;}

protected:

    std::string topic_name_;
};

template <typename data_t, typename msg_t>
class RealTimePublisherBase : public RealTimePublisherInterface
{
public:

    typedef realtime_tools::RealtimePublisher<msg_t> rt_publisher_t;

    RealTimePublisherBase(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
    {
        // Checks
        assert(topic_name.size() > 0);
        assert(data);
        topic_name_ = topic_name;
        pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
        data_ = data;
    }

    /** Publish the topic. */
    inline virtual void publish(const ros::Time& time) = 0;

    inline const data_t* getDataPtr(){return data_;}

    inline rt_publisher_t* getPubPtr(){return pub_ptr_.get();}

protected:
    std::shared_ptr<rt_publisher_t > pub_ptr_;
    data_t* data_ = nullptr;

};

template<typename data_t> struct IsEigen     : std::is_base_of<Eigen::MatrixBase<typename std::decay<data_t>::type>, typename std::decay<data_t>::type > { };
template<typename data_t> struct IsScalar    : std::is_scalar<typename std::decay<data_t>::type> { };

template<typename ...>
using to_void = void;

template<typename T, typename = void>
struct isStdContainer : std::false_type
{};

template<typename T>
struct isStdContainer<T,
        to_void<decltype(std::declval<T>().begin()),
                decltype(std::declval<T>().end()),
                typename T::value_type
        >> : std::true_type // It is enabled for iterable objects
{};

template <typename data_t>
class RealTimePublisher;

template <typename data_t, typename std::enable_if<IsEigen<data_t>::value,int>::type = 0>
inline void resize_imp(RealTimePublisher<data_t>* obj)
{
    unsigned int cols = obj->getDataPtr()->cols();
    unsigned int rows = obj->getDataPtr()->rows();
    obj->getPubPtr()->msg_.array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getPubPtr()->msg_.array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getPubPtr()->msg_.array.layout.dim[0].label = "rows";
    obj->getPubPtr()->msg_.array.layout.dim[1].label = "cols";
    obj->getPubPtr()->msg_.array.layout.dim[0].size = rows;
    obj->getPubPtr()->msg_.array.layout.dim[1].size = cols;
    obj->getPubPtr()->msg_.array.layout.dim[0].stride = rows*cols;
    obj->getPubPtr()->msg_.array.layout.dim[1].stride = cols;
    obj->getPubPtr()->msg_.array.layout.data_offset = 0;
    obj->getPubPtr()->msg_.array.data.resize(rows*cols);
}

template <typename data_t, typename std::enable_if<IsScalar<data_t>::value,int>::type = 1>
inline void resize_imp(RealTimePublisher<data_t>* obj)
{
    obj->getPubPtr()->msg_.array.data.resize(1);
}

template <typename data_t, typename std::enable_if<isStdContainer<data_t>::value,int>::type = 2>
inline void resize_imp(RealTimePublisher<data_t>* obj)
{
    obj->getPubPtr()->msg_.array.data.resize(obj->getDataPtr()->size());
}

template <typename data_t, typename std::enable_if<IsEigen<data_t>::value,int>::type = 0>
inline void publish_imp(RealTimePublisher<data_t>* obj, const ros::Time& time)
{
    if(obj->getPubPtr()->trylock() && obj->getDataPtr())
    {
        const unsigned int & cols = obj->getDataPtr()->cols();
        const unsigned int & rows = obj->getDataPtr()->rows();
        for(unsigned int i = 0; i < rows; i++)
            for(unsigned int j = 0; j < cols; j++)
                obj->getPubPtr()->msg_.array.data[i*cols + j] = static_cast<float>(obj->getDataPtr()->operator()(i,j));
        obj->getPubPtr()->msg_.time.data = time;
        obj->getPubPtr()->unlockAndPublish();
    }
}

template <typename data_t, typename std::enable_if<IsScalar<data_t>::value,int>::type = 1>
inline void publish_imp(RealTimePublisher<data_t>* obj, const ros::Time& time)
{
    if(obj->getPubPtr()->trylock() && obj->getDataPtr())
    {
        obj->getPubPtr()->msg_.array.data[0] = static_cast<float>(*obj->getDataPtr());
        obj->getPubPtr()->msg_.time.data = time;
        obj->getPubPtr()->unlockAndPublish();
    }
}

template <typename data_t, typename std::enable_if<isStdContainer<data_t>::value,int>::type = 2>
inline void publish_imp(RealTimePublisher<data_t>* obj, const ros::Time& time)
{
    if(obj->getPubPtr()->trylock() && obj->getDataPtr())
    {
        const unsigned int & size = obj->getDataPtr()->size();
        for(unsigned int i = 0; i < size; i++)
                obj->getPubPtr()->msg_.array.data[i] = static_cast<float>(obj->getDataPtr()->operator[](i));
        obj->getPubPtr()->msg_.time.data = time;
        obj->getPubPtr()->unlockAndPublish();
    }
}

template <typename data_t>
class RealTimePublisher : public RealTimePublisherBase<data_t,rt_logger::LoggerNumeric>
{
public:

    RealTimePublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
        :RealTimePublisherBase<data_t,rt_logger::LoggerNumeric>(ros_nh,topic_name,data)
    {
        resize_imp<data_t>(this);
    }

    /** Publish the topic. */
    inline void publish(const ros::Time& time) override
    {
        publish_imp<data_t>(this,time);
    }
};

class RealTimePublishers
{
public:

    /**
     * @brief Shared pointer to RealTimePublishers
     */
    typedef std::shared_ptr<RealTimePublishers> Ptr;

    /**
     * @brief Shared pointer to const RealTimePublishers
     */
    typedef std::shared_ptr<const RealTimePublishers> ConstPtr;

    typedef RealTimePublisherInterface rt_publisher_interface_t;

    RealTimePublishers(const ros::NodeHandle& ros_nh)
    {
        nh_ = ros_nh;
    }

    // Add a RealTimePublisher already created
    void addPublisher(std::shared_ptr<rt_publisher_interface_t> pub_ptr)
    {
        assert(pub_ptr);
        // Put it into the map with its friends
        map_[pub_ptr->getTopic()] = pub_ptr;
    }

    // Add a new fresh RealTimePublisher
    template <typename data_t>
    void addPublisher(const std::string& topic_name, data_t* const data_ptr)
    {
        std::shared_ptr<RealTimePublisher<data_t>> new_pub_ptr;
        new_pub_ptr.reset(new RealTimePublisher<data_t>(nh_,topic_name,data_ptr));

        std::shared_ptr<rt_publisher_interface_t> pub_ptr =
                std::static_pointer_cast<rt_publisher_interface_t>(new_pub_ptr);
        addPublisher(pub_ptr);
    }

    // Publish!
    void publishAll(const ros::Time& time)
    {
        for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
            iterator->second->publish(time);
    }

protected:

    typedef std::map<std::string,std::shared_ptr<rt_publisher_interface_t> > pubs_map_t;
    typedef typename pubs_map_t::iterator pubs_map_it_t;

    ros::NodeHandle nh_;
    pubs_map_t map_;
};


} // namespace


#endif
