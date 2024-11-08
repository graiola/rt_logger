#ifndef RT_LOGGER_PUBLISHERS_H
#define RT_LOGGER_PUBLISHERS_H

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <rt_logger/msg/logger_numeric_array.hpp>
#include <eigen3/Eigen/Core>
#include <type_traits>
#include <regex>
#include <mutex>

namespace rt_logger
{

class MsgInterface
{
public:
  using Ptr = std::shared_ptr<MsgInterface>;
  using ros_msg_t = rt_logger::msg::LoggerNumeric;

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
  RealTimePublisherInterface() {}
  virtual ~RealTimePublisherInterface() {}

  virtual void publish(const rclcpp::Time& time) = 0;
  virtual bool addMsg(MsgInterface::Ptr msg) = 0;
  inline const std::string& getTopic() {return topic_name_;}

protected:
  std::string topic_name_;
};

class RealTimePublisher : public RealTimePublisherInterface
{
public:
  using ros_msg_array_t = rt_logger::msg::LoggerNumericArray;
  using rt_publisher_t = realtime_tools::RealtimePublisher<ros_msg_array_t>;

  RealTimePublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic_name)
  {
    assert(!topic_name.empty());
    topic_name_ = topic_name;
    auto pub_ptr = node->create_publisher<ros_msg_array_t>(topic_name, rclcpp::QoS(10));
    pub_ptr_ = std::make_shared<rt_publisher_t>(pub_ptr);
  }

  inline void publish(const rclcpp::Time& time) override
  {
    if (pub_ptr_->trylock())
    {
      unsigned int idx = 0;
      for (auto& tmp_map : msgs_map_)
      {
        tmp_map.second->fillMsg();
        pub_ptr_->msg_.array[idx] = tmp_map.second->getRosMsg();
        idx++;
      }
      pub_ptr_->msg_.time = time;
      pub_ptr_->unlockAndPublish();
    }
  }

  virtual bool addMsg(MsgInterface::Ptr msg) override
  {
    pub_ptr_->msg_.array.push_back(msg->getRosMsg());
    if (msgs_map_.count(msg->getName()) != 0)
      return false;

    msgs_map_[msg->getName()] = msg;
    return true;
  }

  inline rt_publisher_t* getPubPtr() {return pub_ptr_.get();}

protected:
  std::shared_ptr<rt_publisher_t> pub_ptr_;
  std::map<std::string, MsgInterface::Ptr> msgs_map_;
};

// Template specializations (same as before)
template<typename data_t> struct IsEigen : std::is_base_of<Eigen::MatrixBase<typename std::decay<data_t>::type>, typename std::decay<data_t>::type> {};
template<typename data_t> struct IsScalar : std::is_scalar<typename std::decay<data_t>::type> {};

template<typename ...> using to_void = void;

template<typename T, typename = void> struct isStdContainer : std::false_type {};
template<typename T> struct isStdContainer<T, to_void<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end()), typename T::value_type>> : std::true_type {};

template <typename data_t, typename std::enable_if<IsEigen<data_t>::value,int>::type = 0>
inline void resize_imp(Msg<data_t>* obj)
{
  unsigned int rows = obj->getDataPtr()->rows();
  unsigned int cols = obj->getDataPtr()->cols();
  obj->getRosMsg().array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  obj->getRosMsg().array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  obj->getRosMsg().array.layout.dim[0].label = "rows";
  obj->getRosMsg().array.layout.dim[1].label = "cols";
  obj->getRosMsg().array.layout.dim[0].size = rows;
  obj->getRosMsg().array.layout.dim[1].size = cols;
  obj->getRosMsg().array.layout.dim[0].stride = cols;
  obj->getRosMsg().array.layout.dim[1].stride = 1;
  obj->getRosMsg().array.layout.data_offset = 0;
  obj->getRosMsg().array.data.resize(rows * cols);
}

template <typename data_t, typename std::enable_if<IsScalar<data_t>::value, int>::type = 1>
inline void resize_imp(Msg<data_t>* obj)
{
  obj->getRosMsg().array.layout.dim.emplace_back(std_msgs::msg::MultiArrayDimension());
  obj->getRosMsg().array.layout.dim.emplace_back(std_msgs::msg::MultiArrayDimension());
  obj->getRosMsg().array.layout.dim[0].label = "rows";
  obj->getRosMsg().array.layout.dim[1].label = "cols";
  obj->getRosMsg().array.layout.dim[0].size = 1;
  obj->getRosMsg().array.layout.dim[1].size = 1;
  obj->getRosMsg().array.layout.dim[0].stride = 1;
  obj->getRosMsg().array.layout.dim[1].stride = 1;
  obj->getRosMsg().array.layout.data_offset = 0;
  obj->getRosMsg().array.data.resize(1);
}

template <typename data_t, typename std::enable_if<isStdContainer<data_t>::value,int>::type = 2>
inline void resize_imp(Msg<data_t>* obj)
{
  unsigned int rows = obj->getDataPtr()->size(); // We assume a column vector
  unsigned int cols = 1;
  obj->getRosMsg().array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  obj->getRosMsg().array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
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
  using Ptr = std::shared_ptr<PublishersManager>;
  using ConstPtr = std::shared_ptr<const PublishersManager>;
  using rt_publisher_interface_t = RealTimePublisherInterface;

  PublishersManager(const rclcpp::Node::SharedPtr& node) : node_(node) {}

  template <typename data_t>
  void addPublisher(const std::string& topic_name, data_t* const data_ptr, const std::string& data_name = "")
  {
    std::lock_guard<std::mutex> lock(pubs_map_mutex_);
    std::string name = data_name.empty() ? std::regex_replace(topic_name, std::regex("/"), "_").substr(topic_name.find_first_of("_") + 1) : data_name;

    auto new_msg_ptr = std::make_shared<Msg<data_t>>(data_ptr, name);
    MsgInterface::Ptr msg_ptr = std::static_pointer_cast<MsgInterface>(new_msg_ptr);

    if (pubs_map_.count(topic_name) == 0)
      createPublisher(topic_name);

    if (!pubs_map_[topic_name]->addMsg(msg_ptr))
      RCLCPP_WARN(node_->get_logger(), "Cannot add Msg %s: name already taken!", name.c_str());
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
  void publish(const rclcpp::Time& time)
  {
    for (auto& tmp_map : pubs_map_)
      tmp_map.second->publish(time);
  }

  /**
     * @brief Publish the selected topic
     */
  void publish(const rclcpp::Time& time, const std::string& topic_name)
  {
    if(pubs_map_.count(topic_name))
      pubs_map_[topic_name]->publish(time);
  }

private:
  void createPublisher(const std::string& topic_name)
  {
    auto new_pub_ptr = std::make_shared<RealTimePublisher>(node_, topic_name);
    pubs_map_[topic_name] = std::static_pointer_cast<rt_publisher_interface_t>(new_pub_ptr);
  }

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::shared_ptr<rt_publisher_interface_t>> pubs_map_;
  std::mutex pubs_map_mutex_;
};

} // namespace rt_logger

#endif

