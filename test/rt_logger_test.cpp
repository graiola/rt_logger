#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rt_logger/rt_logger.h>

using namespace rt_logger;

std::vector<double> std_vector(3);
double scalar;
Eigen::VectorXd eigen_vector(10);
Eigen::MatrixXd eigen_matrix(10,10);

TEST(RtLoggerTest, StdVector)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/std_vector",std_vector));
}

TEST(RtLoggerTest, Scalar)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/scalar",scalar));
}

TEST(RtLoggerTest, EigenVector)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/eigen_vector",eigen_vector));
}

TEST(RtLoggerTest, EigenMatrix)
{
    EXPECT_NO_THROW(RtLogger::getLogger().addPublisher("/eigen_matrix",eigen_matrix));
}

TEST(RtLoggerTest, Publish)
{
    int cnt = 0;
    while(ros::ok() && cnt++<100)
    {
        EXPECT_NO_THROW(RtLogger::getLogger().publish(ros::Time::now()));
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
