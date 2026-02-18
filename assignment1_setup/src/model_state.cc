/*
 * model_state.cc
 * Copyright (C) 2022 Morgan McColl <morgan.mccoll@alumni.griffithuni.edu.au>
 *
 * Distributed under terms of the MIT license.
 */

#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "assignment1_setup/ModelState.h"
#include "noise.h"
#include "geometry_msgs/Vector3.h"

ros::ServiceClient getModelStateClient;

bool getModelState(assignment1_setup::ModelState::Request &req, assignment1_setup::ModelState::Response &res) {
    gazebo_msgs::GetModelState gazeboSrv;
    gazeboSrv.request.model_name = "turtlebot3_burger";
    if (!getModelStateClient.call(gazeboSrv)) {
        return false;
    }
    res.header = gazeboSrv.response.header;
    res.success = gazeboSrv.response.success;
    res.status_message = gazeboSrv.response.status_message;
    double xStdDev = 5.0;
    double yStdDev = 10.0;
    double x0 = gazeboSrv.response.pose.position.x * 100.0;
    double y0 = gazeboSrv.response.pose.position.y * 100.0;
    double x1 = addNoise(x0, xStdDev);
    double y1 = addNoise(y0, yStdDev);
    double xVariance = xStdDev * xStdDev;
    double yVariance = yStdDev * yStdDev;
    double xyVariance = 4.0;
    geometry_msgs::Vector3 xCovariance;
    xCovariance.x = xVariance;
    xCovariance.y = xyVariance;
    xCovariance.z = 0.0;
    geometry_msgs::Vector3 yCovariance;
    yCovariance.x = xyVariance;
    yCovariance.y = yVariance;
    yCovariance.z = 0.0;
    geometry_msgs::Vector3 zCovariance;
    res.covariance.push_back(xCovariance);
    res.covariance.push_back(yCovariance);
    res.covariance.push_back(zCovariance);
    res.location.x = x1;
    res.location.y = y1;
    res.location.z = gazeboSrv.response.pose.position.z;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "model_state_node");
    ros::NodeHandle n;
    getModelStateClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceServer service = n.advertiseService("turtlebot_position", getModelState);
    ros::spin();
    return EXIT_SUCCESS;
}

