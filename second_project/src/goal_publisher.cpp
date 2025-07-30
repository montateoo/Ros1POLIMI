#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x;
    double y;
    double yaw;
};

std::vector<Goal> readGoalsFromCSV(const std::string& filename) {
    std::vector<Goal> goals;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Could not open file: %s", filename.c_str());
        return goals;
    }

    std::string line;
       while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, yaw_str;
        if (std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',') &&
            std::getline(ss, yaw_str, ',')) {
            Goal g;
            g.x = std::stod(x_str);
            g.y = std::stod(y_str);
            g.yaw = std::stod(yaw_str);
            goals.push_back(g);
        }
    }
    return goals;
}

move_base_msgs::MoveBaseGoal createGoalMsg(const Goal& g) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = g.x;
    goal.target_pose.pose.position.y = g.y;
    goal.target_pose.pose.position.z = 0.0;

    tf::Quaternion q = tf::createQuaternionFromYaw(g.yaw);
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    return goal;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_node");

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun second_project goal_publisher_node <goals.csv>");
        return 1;
    }

    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base action server.");

    std::vector<Goal> goals = readGoalsFromCSV(argv[1]);
    if (goals.empty()) {
        ROS_ERROR("No goals loaded.");
        return 1;
    }

    for (size_t i = 0; i < goals.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal_msg = createGoalMsg(goals[i]);
        ROS_INFO("Sending goal %lu: x=%.2f, y=%.2f, yaw=%.2f", i+1, goals[i].x, goals[i].y, goals[i].yaw);
        ac.sendGoal(goal_msg);

        ac.waitForResult();
        auto result = ac.getState();

        if (result == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %lu reached.", i+1);
        } else {
            ROS_WARN("Goal %lu not reached. Status: %s", i+1, result.toString().c_str());
        }

        ros::Duration(1.0).sleep(); // Optional delay between goals
    }

    ROS_INFO("All goals processed.");
    return 0;
}
