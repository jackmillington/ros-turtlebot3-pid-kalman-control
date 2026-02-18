#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <assignment1_setup/Sonars.h>
#include <assignment1_setup/ModelState.h>
#include <gazebo_msgs/GetModelState.h> 
#include <algorithm>
#include <limits>
#include <cmath>


static ros::Publisher cmdPub;
static ros::ServiceClient posClient;
static ros::ServiceClient gazeboClient;  
static double prevError = 0.0;
static double integral = 0.0;
static int lastIndex = -1;
static ros::Time prevTime;

// Kalman filter init
static bool kalmanInit = false;
static double R_k = 396.956;  // calculated sensor variance in cm²
static double P_k; // estimate variance (cm²)
static double y_k; // filtered distance (cm)

void sonarCallback(const assignment1_setup::Sonars::ConstPtr& msg) {
    
    uint16_t distances[6] {
        msg->distance0,
        msg->distance1,
        msg->distance2,
        msg->distance3,
        msg->distance4,
        msg->distance5
    };

    ros::Time now = ros::Time::now();
    if (prevTime.is_zero()) {
        // This is the first time the callback is called.
        prevTime = now;
        return;
    }
    double dt = (now - prevTime).toSec();
    prevTime = now;

    if (dt <= 0.0){return;} // stops dt / 0 

    // find the closest sonar reading:
    int currentIndex = -1;
    double rawDist = 65535.0;
    for (int i = 0; i < 6; i++) {
        if (distances[i] < rawDist) {
            rawDist      = distances[i];
            currentIndex = i;
            lastIndex = i;
        }
    }

    // Turning
    if (currentIndex != 1) {
        geometry_msgs::Twist turn;
        turn.linear.x = 0;
        if (currentIndex == 1) {
            turn.angular.z = 0;
        } else if (currentIndex == 0) {
            turn.angular.z = 0.4;
            turn.linear.x = 0.4;
        } else if (currentIndex == 2) {
            turn.angular.z = -0.4;
            turn.linear.x = 0.4;
        } else if (currentIndex == 3) {
            turn.angular.z = 0.5;
        } else if (currentIndex == 4) {
            turn.angular.z = -0.5;
        } else if (currentIndex == 5) {
            turn.angular.z = 0.9;
        } else if (lastIndex == 3) {
            turn.angular.z = 0.3;
        } else if (lastIndex == 4) {
            turn.angular.z = -0.3;
        } else if (lastIndex == 5) {
            turn.angular.z = 0.6;
        } else {
            turn.angular.z = 0.5;
        }
        cmdPub.publish(turn);
        return;
    }

    if (rawDist >= 65535) {
        return;
    }

    // --- TASK 2: Kalman predict/update ---
    
    if (!kalmanInit) {
        y_k        = rawDist;
        P_k        = R_k;
        kalmanInit = true;
    }
    // 1) Do we have a valid front‐sonar measurement?
    bool haveMeas = (currentIndex == 1 && rawDist < 65535.0);

    // 2) PREDICT step (always)
    gazebo_msgs::GetModelState g_srv;
    g_srv.request.model_name           = "unit_box";
    g_srv.request.relative_entity_name = "turtlebot3_burger";
    double y_pred = y_k;
    double robot_x_cm;
    double robot_y_cm;
    if (gazeboClient.call(g_srv)) {
        robot_x_cm = g_srv.response.pose.position.x * 100.0;
        robot_y_cm = g_srv.response.pose.position.y * 100.0;
        double predictedDist = std::hypot(robot_x_cm, robot_y_cm);
        y_pred = predictedDist;
    } else {
        ROS_WARN("Gazebo get_model_state service failed");
    }

    // Task 3: fetch & convert covariance from turtlebot_position (m² → cm²)
    assignment1_setup::ModelState cov_srv;
    posClient.call(cov_srv);

    double cov_xx = cov_srv.response.covariance[0].x;
    double cov_xy = cov_srv.response.covariance[0].y;
    double cov_yx = cov_srv.response.covariance[1].x;
    double cov_yy = cov_srv.response.covariance[1].y;

    double Jx = robot_x_cm / y_pred;
    double Jy = robot_y_cm / y_pred;

    double Q_service = Jx*Jx*cov_xx + Jx*Jy*cov_xy + Jx*Jy*cov_yx + Jy*Jy*cov_yy;

    double Q_motion    = std::pow(22.0 * dt, 2);
    double Q           = Q_service + Q_motion; 
    double P_pred      = P_k + Q;
    ROS_INFO("Q_service=%.1f  Q_motion=%.3f  P_pred=%.1f  R_k=%.1f  K=%.3f",
        Q_service, Q_motion, P_pred, R_k,
        P_pred/(P_pred + R_k));

    // 3) UPDATE step (only if front‐sonar)
    if (haveMeas) {
        double K_gain  = P_pred / (P_pred + R_k);
        ROS_INFO("Kalman gain K=%.3f  P_pred=%.1f  R_k=%.1f", K_gain, P_pred, R_k);
        y_k            = y_pred + K_gain * (rawDist - y_pred);
        P_k            = (1.0 - K_gain) * P_pred;
    } else {
        // no new measurement → carry prediction forward
        y_k = y_pred;
        P_k = P_pred;
    }

    // replace raw reading with filtered value
    double minDist = y_k;
    std::cout << "Filtered distance: " << minDist << std::endl;

    // APPROACH

    double desiredSetpoint = 15;
    double error = minDist - desiredSetpoint; // error is the distance away from desires setpoint
    double derivative = (error - prevError) / dt;
    integral += error * dt;

    double kp = 0.0025; // Tweak as needed
    double kd = 0.00008;
    double ki = 0.000000000;

    double output = kp * error + kd * derivative + ki * integral;

    // Clamp speed to max speed and non negative
    double minSpeed = 0.0;
    double maxSpeed = 5.0;
    output = std::max(minSpeed, std::min(output, maxSpeed));

        
    geometry_msgs::Twist approach;
    approach.linear.x = output;
    
    cmdPub.publish(approach);

    prevError = error;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    cmdPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    gazeboClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ROS_INFO("Waiting for /gazebo/get_model_state...");
    ros::service::waitForService("/gazebo/get_model_state", ros::Duration(5.0));
    posClient = nh.serviceClient<assignment1_setup::ModelState>("turtlebot_position");

    ros::Subscriber sonarSub = nh.subscribe<assignment1_setup::Sonars>(
        "/sonars",
        10,
        sonarCallback
    );

    ros::spin();
}

// Run node: rosrun assignment1_setup controller
// rosrun assignment1_setup model_state