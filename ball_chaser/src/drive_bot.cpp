#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBotNode
{
    // ROS::NodeHandle
    ros::NodeHandle node;

    // ROS::Publisher motor commands;
    ros::Publisher motor_command_publisher;

    // ROS::ServiceServer command robot;
    ros::ServiceServer command_robot_service;

    // handle drive_bot service requests
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, 
        ball_chaser::DriveToTarget::Response& res)
    {
        // Publish the requested linear x and angular velocities to the robot wheel joints
        geometry_msgs::Twist motor_command;

        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        motor_command_publisher.publish(motor_command);

        // Return response message
        res.msg_feedback = "Wheel joints velocity set - linear x: " + std::to_string(req.linear_x) + " , angular z: " + std::to_string(req.angular_z);

        return true;
    }


public:
    DriveBotNode()
        : node() // initialize ROS NodeHandle object
        , motor_command_publisher(node.advertise<geometry_msgs::Twist>("/cmd_vel", 10)) // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        , command_robot_service(node.advertiseService("/ball_chaser/command_robot", &DriveBotNode::handle_drive_request, this)) // Create a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    {
    }
};

template <typename _NodeImpl>
void RunNodeSync(int argc, char** argv, const char* node_name)
{
    ros::init(argc, argv, node_name);
    _NodeImpl node;
    ros::spin();
}

int main(int argc, char** argv)
{
    RunNodeSync<DriveBotNode>(argc, argv, "drive_bot");
    return 0;
}
