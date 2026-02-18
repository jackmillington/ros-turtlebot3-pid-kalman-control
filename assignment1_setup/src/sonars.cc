/*
 * sonars.cc
 * Copyright (C) 2021 Morgan McColl
 *
 * Distributed under terms of the MIT license.
 */

#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "assignment1_setup/Sonars.h"
#include "geometry_msgs/Pose.h"
#include <math.h>
#include <stdint.h>
#include <string>
#include <algorithm>

#ifdef NOISY_SONAR
#include "noise.h"
#include "assignment1_setup/ModelState.h"
#endif

bool isSeen(double angle) { // Returns true if seen by sonars
    return (angle <= 60.0 && angle >= -60.0) || 
           (angle <= -70 && angle >= -110) ||
           (angle <= 110 && angle >= 70) ||
           ((angle <= -160 && angle >= - 180) || (angle >= 160 && angle <= 180));
}
bool isZero(double angle) {
    return angle <= 60.0 && angle > 20.0;
}
bool isFirst(double angle) {
    return angle <= 20.0 && angle > -20.0;
}
bool isSecond(double angle) {
    return angle <= -20.0 && angle >= -60.0;
}
bool isThird(double angle) {
    return angle <= 110.0 && angle >= 70.0;
}
bool isFourth(double angle) {
    return angle <= -70.0 && angle >= -110.0;
}
bool isFifth(double angle) {
    return (angle <= -160 && angle >= - 180) || (angle >= 160 && angle <= 180);
}

assignment1_setup::Sonars setMsg(uint16_t distance0, uint16_t distance1, uint16_t distance2, uint16_t distance3, uint16_t distance4, uint16_t distance5) {
    assignment1_setup::Sonars msg;
    msg.distance0 = distance0;
    msg.distance1 = distance1;
    msg.distance2 = distance2;
    msg.distance3 = distance3;
    msg.distance4 = distance4;
    msg.distance5 = distance5;
    return msg;
}

int main(int argc, char **argv) {
    // Initialize ROS and create a node
    ros::init(argc, argv, "sonars");
    ros::NodeHandle nhPrivate("~");
    ros::NodeHandle nh;

    // Retrieve target and robot names from parameters (with defaults)
    std::string targetName = "unit_box";
    std::string robotName  = "turtlebot3_burger";
    nhPrivate.getParam("target", targetName);
    nhPrivate.getParam("robot", robotName);

    // Log a startup message so we know it's running
    ROS_INFO("Sonars node started. Target='%s', Robot='%s'", targetName.c_str(), robotName.c_str());

    // Publisher for the /sonars topic
    ros::Publisher pub = nh.advertise<assignment1_setup::Sonars>("sonars", 1000);

    // Service client for /gazebo/get_model_state
    ros::ServiceClient locations =
        nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // Define the request structure
    gazebo_msgs::GetModelState srv;
    srv.request.model_name           = targetName;
    srv.request.relative_entity_name = robotName;

    ros::Rate rate(100);

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();

        // Wait briefly for the service to be available
        if (!ros::service::waitForService("/gazebo/get_model_state", ros::Duration(5.0))) {
            ROS_ERROR("Service /gazebo/get_model_state not available.");
            return EXIT_FAILURE;
        }

        // Call the service to get the relative position
        if (!locations.call(srv)) {
            ROS_ERROR("Failed to query model state. Is Gazebo running?");
            rate.sleep();
            continue;
        }

        // If the target doesn't exist or can't be found
        if (!srv.response.success) {
            ROS_ERROR("Can't find target object '%s' for robot '%s'.", 
                      targetName.c_str(), robotName.c_str());
            rate.sleep();
            continue;
        }

        // Calculate distance and angle from robot to target
        double x     = srv.response.pose.position.x;
        double y     = srv.response.pose.position.y;
        double angle = atan2(y, x) * (180.0 / M_PI);

        // Create and publish the Sonars message
        assignment1_setup::Sonars msg;
        if (!isSeen(angle)) {
            // No detection in ±45° FOV
            msg = setMsg(UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
            pub.publish(msg);
            ROS_INFO("Published Sonar (no detection): [%u, %u, %u, %u, %u, %u]",
                     msg.distance0, msg.distance1, msg.distance2, msg.distance3, msg.distance4, msg.distance5);
            rate.sleep();
            continue;
        }

#ifdef NOISY_SONAR
        double centimetres = addNoise(sqrt(x * x + y * y) * 100.0, 20.0);
#else
        double centimetres = sqrt(x * x + y * y) * 100.0;
#endif

        // Clamp distance to valid uint16 range
        double clamped    = std::max(std::min(centimetres, 65535.0), 0.0);
        uint16_t distance = static_cast<uint16_t>(clamped);

        // Decide which of the three sensors sees the target
        if (isZero(angle)) {
            msg = setMsg(distance, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
        } else if (isFirst(angle)) {
            msg = setMsg(UINT16_MAX, distance, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
        } else if (isSecond(angle)) {
            msg = setMsg(UINT16_MAX, UINT16_MAX, distance, UINT16_MAX, UINT16_MAX, UINT16_MAX);
        } else if (isThird(angle)) {
            msg = setMsg(UINT16_MAX, UINT16_MAX, UINT16_MAX, distance, UINT16_MAX, UINT16_MAX);
        } else if (isFourth(angle)) {
            msg = setMsg(UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, distance, UINT16_MAX);
        } else if (isFifth(angle)) {
            msg = setMsg(UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, distance);
        }

        pub.publish(msg);
        ROS_INFO("Published Sonar: [%u, %u, %u, %u, %u, %u]",
                 msg.distance0, msg.distance1, msg.distance2, msg.distance3, msg.distance4, msg.distance5);

        rate.sleep();
    }

    return EXIT_SUCCESS;
}

// run world: roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
// Run Cmd: rosrun assignment1_setup sonars _target:=unit_box _robot:=turtlebot3_burger