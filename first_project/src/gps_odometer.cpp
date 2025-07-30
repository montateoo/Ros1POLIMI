#include <ros/ros.h>
#include <deque>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

const double a = 6378137;
const double b = 6356752;

struct ENU {
    double x;
    double y;
    double z;
};

class GPSOdometerNode {
public:
    GPSOdometerNode() {
        pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 1, &GPSOdometerNode::callback, this);

        //Default value (not used until reference is set)
        current_value_.header.stamp = ros::Time::now();
        current_value_.header.frame_id = "odom";
    }

    double N(double x){
        return a/sqrt(1 - e_2 * pow(sin(x), 2));
    }

    bool isInitialized() const {
        return reference_initialized_;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    sensor_msgs::NavSatFix gps_to_odom;
    sensor_msgs::NavSatFix current_value_;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    double x0_, y0_, z0_;
    double sinLat_, cosLat_, sinLon_, cosLon_;
    double e_2 = 1 - (pow(b, 2) / pow(a, 2));

    bool reference_initialized_ = false;

    std::deque<ENU> enu_history_;
    const size_t history_size_ = 10;  // numero di posizioni nella coda
    const double min_movement_threshold = 0.01;

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

    //filtro per posizioni gps non valide
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        (msg->latitude == 0.0 || msg->longitude == 0.0 || msg->altitude == 0.0)) {
        //ROS_WARN("GPS data ignored: invalid fix or null coordinates [ %f , %f , %f ]", msg->latitude, msg->longitude, msg->altitude);
        return;
    }

        if (!reference_initialized_) {
            gps_to_odom = *msg;
            reference_initialized_ = true;

            // Pre-calcolo coordinate ECEF del punto di riferimento
            double ref_lat_rad = gps_to_odom.latitude * M_PI / 180.0;
            double ref_lon_rad = gps_to_odom.longitude * M_PI / 180.0;
            x0_ = (N(ref_lat_rad) + gps_to_odom.altitude) * cos(ref_lat_rad) * cos(ref_lon_rad);
            y0_ = (N(ref_lat_rad) + gps_to_odom.altitude) * cos(ref_lat_rad) * sin(ref_lon_rad);
            z0_ = (N(ref_lat_rad)*(1 - e_2) + gps_to_odom.altitude) * sin(ref_lat_rad);

            sinLat_ = sin(ref_lat_rad);
            cosLat_ = cos(ref_lat_rad);
            sinLon_ = sin(ref_lon_rad);
            cosLon_ = cos(ref_lon_rad);

            ROS_INFO("Reference GPS position set.");
            ROS_INFO("Latitude: %f", gps_to_odom.latitude);
            ROS_INFO("Longitude: %f", gps_to_odom.longitude);
            ROS_INFO("Altitude: %f", gps_to_odom.altitude);
            return;
        }


    current_value_ = *msg;

    // Conversione da GPS a ENU
    double lat_rad = current_value_.latitude * M_PI / 180.0;  // da deg a rad
    double lon_rad = current_value_.longitude * M_PI / 180.0; // da deg a rad

    double x = (N(lat_rad)+current_value_.altitude)*cos(lat_rad)*cos(lon_rad);
    double y = (N(lat_rad)+current_value_.altitude)*cos(lat_rad)*sin(lon_rad);
    double z = (N(lat_rad)*(1 - e_2) + current_value_.altitude) * sin(lat_rad);

    double dx = x - x0_;
    double dy = y - y0_;
    double dz = z - z0_;

    ENU enu;
    enu.x = -sinLon_ * dx + cosLon_ * dy;
    enu.y = -sinLat_ * cosLon_ * dx - sinLat_ * sinLon_ * dy + cosLat_ * dz;
    enu.z =  cosLat_ * cosLon_ * dx + cosLat_ * sinLon_ * dy + sinLat_ * dz;

    // Costruzione messaggio Odometry
    nav_msgs::Odometry msg_out;

    ros::Time now = ros::Time::now();
    msg_out.header.stamp = now;
    msg_out.header.frame_id = "odom";
    msg_out.child_frame_id = "gps";

    msg_out.pose.pose.position.x = enu.x;
    msg_out.pose.pose.position.y = enu.y;
    msg_out.pose.pose.position.z = enu.z;

    //orientation
    double yaw = 0.0;

    //nuova posizione aggiunta alla coda
    enu_history_.push_back(enu);

    // Se la coda è troppo lunga la posizione meno recente è scartata
    if (enu_history_.size() > history_size_) {
      enu_history_.pop_front();
    }

    if (enu_history_.size() >= 2) {
      ENU oldest = enu_history_.front();  //punto più vecchio
      ENU newest = enu_history_.back();   //punto più recente

      double delta_x = newest.x - oldest.x;
      double delta_y = newest.y - oldest.y;

      if (std::hypot(delta_x, delta_y) > min_movement_threshold) { // movimento significativo
        yaw = atan2(delta_y, delta_x);
      }
    }

    msg_out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);


    // Pose covariance (solo i valori principali)
    std::fill(std::begin(msg_out.pose.covariance), std::end(msg_out.pose.covariance), 0.0);
    msg_out.pose.covariance[0] = 0.05;    // x
    msg_out.pose.covariance[7] = 0.05;    // y
    msg_out.pose.covariance[35] = 0.1;    // yaw

    pub_.publish(msg_out);

    //Pubblico il TF
    transform.setOrigin(tf::Vector3(enu.x, enu.y, enu.z));
    q.setRPY(0, 0, yaw);

    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, now, "odom", "gps"));

}

};

int main(int argc, char** argv) {
    ROS_INFO("gps_odometer node started");
    ros::init(argc, argv, "gps_odometer_node");
    GPSOdometerNode node;
    ros::spin();
    return 0;
}

