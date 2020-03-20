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
#include <rt_logger/LoggerNumericArray.h>
#include <eigen3/Eigen/Core>
#include <type_traits>

namespace rt_logger
{

class MsgInterface
{
public:

    /**
     * @brief Shared pointer to Msg
     */
    typedef std::shared_ptr<MsgInterface> Ptr;

    typedef rt_logger::LoggerNumeric ros_msg_t;

    MsgInterface() {}

    virtual ~MsgInterface() {}

    virtual void fillMsg() = 0;

    inline const std::string& getName() {return name_;}

    inline ros_msg_t& getRosMsg() {return ros_msg_;}

protected:

    std::string name_;

private:

    ros_msg_t ros_msg_;
};

template <typename data_t>
class Msg;

class RealTimePublisherInterface
{
public:

    /** Initialize the real time publisher. */
    RealTimePublisherInterface() {}

    virtual ~RealTimePublisherInterface() {}

    /** Publish the topic. */
    virtual void publish(const ros::Time& time) = 0;

    virtual bool addMsg(MsgInterface::Ptr msg) = 0;

    inline const std::string& getTopic() {return topic_name_;}

protected:

    std::string topic_name_;
};

class RealTimePublisher : public RealTimePublisherInterface // FIXME change names
{
public:

    typedef rt_logger::LoggerNumericArray ros_msg_array_t;

    typedef realtime_tools::RealtimePublisher<ros_msg_array_t> rt_publisher_t;

    RealTimePublisher(const ros::NodeHandle& ros_nh, const std::string topic_name)
    {
        // Checks
        assert(topic_name.size() > 0);
        topic_name_ = topic_name;
        pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
    }

    /** Publish the topic. */
    inline void publish(const ros::Time& time) override
    {
        if(this->getPubPtr()->trylock())
        {   unsigned int idx = 0;
            for (auto& tmp_map : msgs_map_)
            {
                tmp_map.second->fillMsg();
                this->getPubPtr()->msg_.array[idx] = tmp_map.second->getRosMsg();
                idx++;
            }
            this->getPubPtr()->msg_.time.data = time;
            this->getPubPtr()->unlockAndPublish();
        }
    }


    virtual bool addMsg(MsgInterface::Ptr msg) override
    {
        this->getPubPtr()->msg_.array.push_back(msg->getRosMsg());
        if(msgs_map_.count(msg->getName())!=0)
            return false;

        msgs_map_[msg->getName()] = msg;
        return true;
    }

    inline rt_publisher_t* getPubPtr(){return pub_ptr_.get();}

protected:
    std::shared_ptr<rt_publisher_t > pub_ptr_;
    std::map<std::string,MsgInterface::Ptr> msgs_map_;

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

template <typename data_t, typename std::enable_if<IsEigen<data_t>::value,int>::type = 0>
inline void resize_imp(Msg<data_t>* obj)
{
    unsigned int rows = obj->getDataPtr()->rows();
    unsigned int cols = obj->getDataPtr()->cols();
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim[0].label = "rows";
    obj->getRosMsg().array.layout.dim[1].label = "cols";
    obj->getRosMsg().array.layout.dim[0].size = rows;
    obj->getRosMsg().array.layout.dim[1].size = cols;
    obj->getRosMsg().array.layout.dim[0].stride = cols;
    obj->getRosMsg().array.layout.dim[1].stride = 1;
    obj->getRosMsg().array.layout.data_offset = 0;
    obj->getRosMsg().array.data.resize(rows*cols);
}

template <typename data_t, typename std::enable_if<IsScalar<data_t>::value,int>::type = 1>
inline void resize_imp(Msg<data_t>* obj)
{
    unsigned int rows = 1;
    unsigned int cols = 1;
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim[0].label = "rows";
    obj->getRosMsg().array.layout.dim[1].label = "cols";
    obj->getRosMsg().array.layout.dim[0].size = rows;
    obj->getRosMsg().array.layout.dim[1].size = cols;
    obj->getRosMsg().array.layout.dim[0].stride = 0;
    obj->getRosMsg().array.layout.dim[1].stride = 0;
    obj->getRosMsg().array.layout.data_offset = 0;
    obj->getRosMsg().array.data.resize(rows*cols);
}

template <typename data_t, typename std::enable_if<isStdContainer<data_t>::value,int>::type = 2>
inline void resize_imp(Msg<data_t>* obj)
{
    unsigned int rows = obj->getDataPtr()->size(); // We assume a column vector
    unsigned int cols = 1;
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    obj->getRosMsg().array.layout.dim[0].label = "rows";
    obj->getRosMsg().array.layout.dim[1].label = "cols";
    obj->getRosMsg().array.layout.dim[0].size = rows;
    obj->getRosMsg().array.layout.dim[1].size = cols;
    obj->getRosMsg().array.layout.dim[0].stride = 1;
    obj->getRosMsg().array.layout.dim[1].stride = 0;
    obj->getRosMsg().array.layout.data_offset = 0;
    obj->getRosMsg().array.data.resize(rows*cols);
}

/*  To access the data the common formulation is the following: v[i_row * stride_row + j_col * stride_col + data_offset] */
template <typename data_t, typename std::enable_if<IsEigen<data_t>::value,int>::type = 0>
inline void fill_msg_imp(Msg<data_t>* obj)
{
    const unsigned int & cols = obj->getDataPtr()->cols();
    const unsigned int & rows = obj->getDataPtr()->rows();
    for(unsigned int i = 0; i < rows; i++)
        for(unsigned int j = 0; j < cols; j++)
            obj->getRosMsg().array.data[i*cols + j] = static_cast<float>(obj->getDataPtr()->operator()(i,j));
}

template <typename data_t, typename std::enable_if<IsScalar<data_t>::value,int>::type = 1>
inline void fill_msg_imp(Msg<data_t>* obj)
{
    obj->getRosMsg().array.data[0] = static_cast<float>(*obj->getDataPtr());
}

template <typename data_t, typename std::enable_if<isStdContainer<data_t>::value,int>::type = 2>
inline void fill_msg_imp(Msg<data_t>* obj)
{
    const unsigned int & rows = obj->getDataPtr()->size();
    for(unsigned int i = 0; i < rows; i++)
        obj->getRosMsg().array.data[i] = static_cast<float>(obj->getDataPtr()->operator[](i));
}

template <typename data_t>
class Msg : public MsgInterface
{
public:

    Msg(data_t* const data, const std::string data_name)
    {
        assert(data);
        data_ = data;
        name_ = data_name;
        resize_imp<data_t>(this);
    }

    virtual void fillMsg() override
    {
        fill_msg_imp<data_t>(this);
        this->getRosMsg().name.data = name_;
    }

    inline const data_t* getDataPtr(){return data_;}

private:

    data_t* data_ = nullptr;
};

class PublishersManager
{
public:

    /**
     * @brief Shared pointer to PublishersManager
     */
    typedef std::shared_ptr<PublishersManager> Ptr;

    /**
     * @brief Shared pointer to const PublishersManager
     */
    typedef std::shared_ptr<const PublishersManager> ConstPtr;

    typedef RealTimePublisherInterface rt_publisher_interface_t;

    PublishersManager(const ros::NodeHandle& ros_nh)
    {
        nh_ = ros_nh;
    }

    /**
     * @brief Add ad new real time publisher
     */
    template <typename data_t>
    void addPublisher(const std::string& topic_name, data_t* const data_ptr, const std::string& data_name = "")
    {
        // Create the new message
        std::shared_ptr<Msg<data_t>> new_msg_ptr;
        new_msg_ptr.reset(new Msg<data_t>(data_ptr,data_name));

        MsgInterface::Ptr msg_ptr =
                std::static_pointer_cast<MsgInterface>(new_msg_ptr);

        // If the topic exists, append the new message to the existing publisher, otherwise create a new publisher
        if(pubs_map_.count(topic_name)==0)
            createPublisher(topic_name);

        if(!pubs_map_[topic_name]->addMsg(msg_ptr))
            ROS_WARN_STREAM("Can not add Msg "<<data_name<< " the name is already taken!");
    }

    /**
     * @brief Publish all the available topics
     */
    void publish(const ros::Time& time)
    {
        for(auto& tmp_map : pubs_map_)
            tmp_map.second->publish(time);
    }

    /**
     * @brief Publish the selected topic
     */
    void publish(const ros::Time& time, const std::string& topic_name)
    {
        if(pubs_map_.count(topic_name))
            pubs_map_[topic_name]->publish(time);
    }

private:

    /**
     * @brief Create a publisher
     */
    void createPublisher(const std::string& topic_name)
    {
        std::shared_ptr<RealTimePublisher> new_pub_ptr;
        new_pub_ptr.reset(new RealTimePublisher(nh_,topic_name));

        std::shared_ptr<rt_publisher_interface_t> pub_ptr =
                std::static_pointer_cast<rt_publisher_interface_t>(new_pub_ptr);

        pubs_map_[topic_name] = pub_ptr;
    }

protected:

    typedef std::map<std::string,std::shared_ptr<rt_publisher_interface_t> > pubs_map_t;
    typedef typename pubs_map_t::iterator pubs_map_it_t;

    ros::NodeHandle nh_;
    pubs_map_t pubs_map_;
};


} // namespace


#endif
