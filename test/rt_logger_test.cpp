#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rt_logger/rt_logger.h>

using namespace rt_logger;

static std::vector<double> std_vector(3);
static double scalar;
static Eigen::VectorXd eigen_vector(10);
static Eigen::MatrixXd eigen_matrix(10,10);

TEST(RtLoggerTest, StdVectorPublisher)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/std_vector",std_vector));
}

TEST(RtLoggerTest, ScalarPublisher)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/scalar",scalar));
}

TEST(RtLoggerTest, EigenVectorPublisher)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/eigen_vector",eigen_vector));
}

TEST(RtLoggerTest, EigenMatrixPublisher)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/eigen_matrix",eigen_matrix));
}

TEST(RtLoggerTest, SinglePublisher)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/single_publisher",std_vector,"std_vector"));
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/single_publisher",scalar,"scalar"));
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/single_publisher",eigen_vector,"eigen_vector"));
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/single_publisher",eigen_matrix,"eigen_matrix"));
}

TEST(RtLoggerTest, PublishAll)
{
    int cnt = 0;
    while(ros::ok() && cnt++<50)
    {
        EXPECT_NO_THROW(RtLogger::getLogger().publish(ros::Time::now()));
        ros::Duration(0.1).sleep();
    }
}

TEST(RtLoggerTest, Publish)
{
    int cnt = 0;
    while(ros::ok() && cnt++<50)
    {
        EXPECT_NO_THROW(RtLogger::getLogger().publish(ros::Time::now(),"/single_publisher"));
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rt_logger_test");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
