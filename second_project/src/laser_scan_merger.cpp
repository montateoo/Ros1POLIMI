#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <limits>
#include <cmath>

class LaserScanMergerNode {
public:
    LaserScanMergerNode() {
        sub_front_ = nh_.subscribe("/scan_front", 1, &LaserScanMergerNode::frontCallback, this);
        sub_back_  = nh_.subscribe("/scan_back", 1, &LaserScanMergerNode::backCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_360", 1);
        nh_.param("min_valid_range", min_valid_range_, 0.3f);
        //nh_.param("wheel_ignore_range", wheel_ignore_range_, 0.6f);


        // Zones to ignore wheel reflections
        wheel_ignore_range_ = {
            { -M_PI * 0.5 - 0.2, -M_PI * 0.5 + 0.2 }, // left wheel area ~[-1.77, -1.37]
            {  M_PI * 0.5 - 0.2,  M_PI * 0.5 + 0.2 }  // right wheel area ~[1.37, 1.77]
        };

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_front_, sub_back_;
    ros::Publisher pub_;
    tf::TransformListener tf_listener_;

    sensor_msgs::LaserScan::ConstPtr front_scan_;
    sensor_msgs::LaserScan::ConstPtr back_scan_;
    bool got_front_ = false, got_back_ = false;

    float min_valid_range_;

    std::vector<std::pair<float, float>> wheel_ignore_range_;

    bool isInWheelIgnoreZone(float angle) {
        for (const auto& zone : wheel_ignore_range_) {
            if (angle >= zone.first && angle <= zone.second) {
                return true;
            }
        }
        return false;
    };

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        front_scan_ = msg;
        got_front_ = true;
        tryPublishMergedScan();
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        back_scan_ = msg;
        got_back_ = true;
        tryPublishMergedScan();
    }

    void tryPublishMergedScan() {
        if (!got_front_ || !got_back_) return;

        float angle_min = -M_PI;
        float angle_max = M_PI;
        float angle_increment = front_scan_->angle_increment;
        int num_readings = std::round((angle_max - angle_min) / angle_increment);

        sensor_msgs::LaserScan merged_scan;
        merged_scan.header.stamp = std::max(front_scan_->header.stamp, back_scan_->header.stamp);
        merged_scan.header.frame_id = "base_link";
        merged_scan.angle_min = angle_min;
        merged_scan.angle_max = angle_max;
        merged_scan.angle_increment = angle_increment;
        merged_scan.range_min = std::min(front_scan_->range_min, back_scan_->range_min);
        merged_scan.range_max = std::max(front_scan_->range_max, back_scan_->range_max);
        merged_scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

        auto insert_scan = [&](const sensor_msgs::LaserScan::ConstPtr& scan, const std::string& from_frame) {
            if (!tf_listener_.waitForTransform("base_link", from_frame, scan->header.stamp, ros::Duration(0.5))) {
                ROS_WARN("TF wait failed for frame: %s", from_frame.c_str());
                return;
            }

            tf::StampedTransform transform;
            try {
                tf_listener_.lookupTransform("base_link", from_frame, scan->header.stamp, transform);
            } catch (tf::TransformException& ex) {
                ROS_WARN("TF error: %s", ex.what());
                return;
            }

            for (size_t i = 0; i < scan->ranges.size(); ++i) {
                float angle = scan->angle_min + i * scan->angle_increment;
                float range = scan->ranges[i];

                if (range < scan->range_min || range > scan->range_max || std::isnan(range))
                    continue;

                float x = range * cos(angle);
                float y = range * sin(angle);

                tf::Vector3 point_in_laser(x, y, 0.0);
                tf::Vector3 point_in_base = transform * point_in_laser;

                float angle_base = atan2(point_in_base.y(), point_in_base.x());
                float range_base = hypot(point_in_base.x(), point_in_base.y());

                int index = std::round((angle_base - angle_min) / angle_increment);
                if (index >= 0 && index < num_readings) {
                    if ((!isInWheelIgnoreZone(angle_base) && range_base >= min_valid_range_)) {
                        if (range_base < merged_scan.ranges[index]) {
                        merged_scan.ranges[index] = range_base;
    }
}
                }
            }
        };

        insert_scan(front_scan_, front_scan_->header.frame_id);
        insert_scan(back_scan_, back_scan_->header.frame_id);

        pub_.publish(merged_scan);
        got_front_ = false;
        got_back_ = false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_merger");
    ROS_INFO("laser_scan_merger_node started");
    LaserScanMergerNode merger;
    ros::spin();
    return 0;
}