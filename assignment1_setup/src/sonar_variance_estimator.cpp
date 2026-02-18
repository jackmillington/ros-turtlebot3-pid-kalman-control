// src/sonar_variance_estimator.cpp

#include <ros/ros.h>
#include <assignment1_setup/Sonars.h>
#include <cmath>

static const int N = 1000;   // number of samples to collect
static int count = 0;
static double mean = 0.0;
static double M2   = 0.0;

// Welford’s online algorithm for variance:
void addSample(double x) {
  count++;
  double delta = x - mean;
  mean += delta / count;
  double delta2 = x - mean;
  M2 += delta * delta2;
}

void sonarCb(const assignment1_setup::Sonars::ConstPtr& msg) {
    
  double z = static_cast<double>(msg->distance1);
  addSample(z);

  if (count >= N) {
    double variance = M2 / (count - 1);  // sample variance
    ROS_INFO("Collected %d samples, estimated variance R = %.3f cm²", count, variance);
    ros::shutdown();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar_variance_estimator");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<assignment1_setup::Sonars>(
    "/sonars", 10, sonarCb);

  ROS_INFO("Estimating variance over %d samples—please hold robot steady.", N);
  ros::spin();
  return 0;
}
