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
#include <regex>
#include <mutex>

#include <deque>    // For circular buffer implementation
#include <cstdlib>  // For accessing system's tmp directory
#include <iomanip>  // For formatting timestamps in filenames
#include <sstream>  // For constructing filenames
#include <matio.h>  // For MAT file operations

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

class RealTimePublisher : public RealTimePublisherInterface
{
public:
  typedef rt_logger::LoggerNumericArray ros_msg_array_t;
  typedef realtime_tools::RealtimePublisher<ros_msg_array_t> rt_publisher_t;

  RealTimePublisher(const ros::NodeHandle& ros_nh, const std::string& topic_name, bool enable_mat_dump = false, size_t buffer_size = 100)
    : enable_mat_dump_(enable_mat_dump), buffer_size_(buffer_size)
  {
    assert(topic_name.size() > 0);
    topic_name_ = topic_name;
    pub_ptr_.reset(new rt_publisher_t(ros_nh, topic_name, 10));
  }

  /** Publish the topic and store data in the circular buffer. */
  inline void publish(const ros::Time& time) override
  {
    if (this->getPubPtr()->trylock())
    {
      unsigned int idx = 0;
      for (auto& tmp_map : msgs_map_)
      {
        tmp_map.second->fillMsg();
        this->getPubPtr()->msg_.array[idx] = tmp_map.second->getRosMsg();
        idx++;
      }
      this->getPubPtr()->msg_.time.data = time;

      // Store data in the circular buffer
      if (enable_mat_dump_)
      {
        buffer_.push_back(this->getPubPtr()->msg_);
        if (buffer_.size() > buffer_size_)
        {
          buffer_.pop_front();
        }
      }

      this->getPubPtr()->unlockAndPublish();
    }
  }

  virtual bool addMsg(MsgInterface::Ptr msg) override
  {
    this->getPubPtr()->msg_.array.push_back(msg->getRosMsg());
    if (msgs_map_.count(msg->getName()) != 0)
      return false;

    msgs_map_[msg->getName()] = msg;
    return true;
  }

  inline rt_publisher_t* getPubPtr() { return pub_ptr_.get(); }

  void dumpToMat()
  {
    if (!enable_mat_dump_)
    {
      ROS_WARN_STREAM("MAT file dump is disabled for this publisher.");
      return;
    }

    // Create a filename with a timestamp
    std::ostringstream filename;
    filename << "/tmp/" << topic_name_ << "_"
             << std::chrono::system_clock::now().time_since_epoch().count()
             << ".mat";

    mat_t* matfile = Mat_CreateVer(filename.str().c_str(), nullptr, MAT_FT_MAT5);
    if (!matfile)
    {
      ROS_ERROR_STREAM("Failed to create MAT file: " << filename.str());
      return;
    }

    size_t entry_idx = 0;

    for (const auto& msg : buffer_)
    {
      for (size_t i = 0; i < msg.array.size(); ++i)
      {
        const auto& logger_numeric = msg.array[i];
        const auto& name = logger_numeric.name.data;
        const auto& array = logger_numeric.array;

        // Extract data and layout from Float64MultiArray
        const auto& data = array.data;
        const auto& layout = array.layout;

        if (layout.dim.size() < 2)
        {
          ROS_ERROR_STREAM("Invalid layout dimensions in LoggerNumeric message.");
          continue;
        }

        const unsigned int rows = layout.dim[0].size;
        const unsigned int cols = layout.dim[1].size;

        if (data.size() != rows * cols)
        {
          ROS_ERROR_STREAM("Data size mismatch in LoggerNumeric message.");
          continue;
        }

        size_t dims[2] = { rows, cols };

        // Use the name from LoggerNumeric for variable naming
        std::string var_name = name.empty() ?
              ("entry_" + std::to_string(entry_idx++) + "_msg_" + std::to_string(i)) :
              name;

        matvar_t* matvar = Mat_VarCreate(
              var_name.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, const_cast<double*>(data.data()), 0);

        if (!matvar)
        {
          ROS_ERROR_STREAM("Failed to create MAT variable for: " << var_name);
          continue;
        }

        if (Mat_VarWrite(matfile, matvar, MAT_COMPRESSION_NONE) != 0)
        {
          ROS_ERROR_STREAM("Failed to write MAT variable for: " << var_name);
        }

        Mat_VarFree(matvar);
      }
    }

    Mat_Close(matfile);
    ROS_INFO_STREAM("Successfully dumped data to MAT file: " << filename.str());
  }



protected:
  std::shared_ptr<rt_publisher_t> pub_ptr_;
  std::map<std::string, MsgInterface::Ptr> msgs_map_;

  // Circular buffer
  std::deque<ros_msg_array_t> buffer_;
  size_t buffer_size_;
  bool enable_mat_dump_;
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
    std::lock_guard<std::mutex> lock(pubs_map_mutex_);
    // If data_name is empty, use the topic_name instead and replace the "/" with "_"
    std::string name;
    if(data_name.empty())
    {
      name = topic_name;
      name = std::regex_replace(name, std::regex("/"), "_");
      name = name.substr(name.find_first_of("_")+1);
    }
    else
      name = data_name;

    // Create the new message
    std::shared_ptr<Msg<data_t>> new_msg_ptr;
    new_msg_ptr.reset(new Msg<data_t>(data_ptr,name));

    MsgInterface::Ptr msg_ptr =
        std::static_pointer_cast<MsgInterface>(new_msg_ptr);

    // If the topic exists, append the new message to the existing publisher, otherwise create a new publisher
    if(pubs_map_.count(topic_name)==0)
      createPublisher(topic_name);

    if(!pubs_map_[topic_name]->addMsg(msg_ptr))
      ROS_WARN_STREAM("Can not add Msg "<<name<< " the name is already taken!");
  }

  /**
     * @brief Remove a real time publisher
     */
  void removePublisher(const std::string& topic_name)
  {
    pubs_map_.erase(topic_name);
  }

  /**
     * @brief Remove all real time publishers
     */
  void removePublishers()
  {
    auto it = pubs_map_.cbegin();
    while (it != pubs_map_.cend())
      it = pubs_map_.erase(it);
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

  std::mutex pubs_map_mutex_;
};


} // namespace


#endif
