#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OdomToTFNode {
public:
    OdomToTFNode() {
        sub_ = nh_.subscribe("/odometry", 1, &OdomToTFNode::callback, this);
        received_first_msg_ = false;
    }

    void spin() {
        ros::Rate rate(10);  // 10 Hz

        while (ros::ok()) {
            if (!received_first_msg_) {
                ROS_WARN_THROTTLE(1.0, "In attesa del primo messaggio su /odometry...");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            ros::Time now = ros::Time::now();

            // Broadcast della trasformazione
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(x_, y_, z_));
            transform.setRotation(q_);

            br_.sendTransform(tf::StampedTransform(transform, msg_time, "odom", "base_link"));

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;
    tf::Quaternion q_;
    double x_, y_, z_;
    bool received_first_msg_;
    ros::Time msg_time;

    void callback(const nav_msgs::Odometry::ConstPtr& msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        msg_time = msg->header.stamp;

        const geometry_msgs::Quaternion& orientation = msg->pose.pose.orientation;
        q_ = tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

        received_first_msg_ = true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf_node");
    ROS_INFO("odom_to_tf_node started");
    OdomToTFNode node;
    node.spin();
    return 0;
}
