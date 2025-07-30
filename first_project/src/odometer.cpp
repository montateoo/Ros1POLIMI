#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

const double L = 1.765;

class OdometerNode {
public:
    OdometerNode() {
        pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
        sub_ = nh_.subscribe("/speedsteer", 1, &OdometerNode::callback, this);
        //subscriber per filtro gps
        gps_sub_ = nh_.subscribe("/gps_odom", 1, &OdometerNode::gpsCallback, this);

        received_first_msg_ = false;

        //Valore iniziale del parametro da pubblicare
        current_value_.header.stamp = ros::Time::now();
        current_value_.header.frame_id = "odom";
        current_value_.point.x = 0;
        current_value_.point.y = 0;
        current_value_.point.z = 0.0;

        // Inizializzazione valori per conversione
        x = 0;
        y = 0;
        theta = M_PI / 2.0;


    }

    void spin() {
    ros::Rate rate(10);  // 10 Hz
    const double steering_factor = 32;  // Fattore di sterzo
    const double kmh_to_ms = 1000.0 / 3600.0;

    while (ros::ok()) {
        if (!received_first_msg_) {
            ROS_WARN_THROTTLE(1.0, "In attesa del primo messaggio su /speedsteer...");
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        if (dt == 0.0) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        last_time_ = now;

        double v = current_value_.point.y * kmh_to_ms;
        double delta = (current_value_.point.x * M_PI) / (180.0 * steering_factor);

        // Validazione input
        if (!std::isfinite(v) || !std::isfinite(delta)) {
          ROS_WARN("Valori non validi: v=%.3f, delta=%.3f", v, delta);
          ros::spinOnce();
          rate.sleep();
          continue;
        }

        // Ignora integrazione se la velocità è troppo bassa
        const double VELOCITY_THRESHOLD = 0.01; // [m/s] da tarare
        if (std::abs(v) < VELOCITY_THRESHOLD) {
          ROS_DEBUG_THROTTLE(5.0, "Velocità troppo bassa (v=%.3f m/s), integrazione saltata.", v);
          ros::spinOnce();
          rate.sleep();
          continue;
        }

        //modello runge-kutta (auto semplificata a bicicletta)
        auto f = [&](double x, double y, double theta) {
            double omega = (v / L) * std::tan(delta);
            return std::make_tuple(
            v * std::cos(theta), // dx/dt
            v * std::sin(theta), // dy/dt
            omega                // dtheta/dt
            );
        };

    // Calcolo di k1
    double k1x, k1y, k1theta;
    std::tie(k1x, k1y, k1theta) = f(x, y, theta);

    // Calcolo di k2
    double k2x, k2y, k2theta;
    std::tie(k2x, k2y, k2theta) = f(x + 0.5 * dt * k1x, y + 0.5 * dt * k1y, theta + 0.5 * dt * k1theta);

    // Calcolo di k3
    double k3x, k3y, k3theta;
    std::tie(k3x, k3y, k3theta) = f(x + 0.5 * dt * k2x, y + 0.5 * dt * k2y, theta + 0.5 * dt * k2theta);

    // Calcolo di k4
    double k4x, k4y, k4theta;
    std::tie(k4x, k4y, k4theta) = f(x + dt * k3x, y + dt * k3y, theta + dt * k3theta);

    // Aggiorna lo stato
    x += (dt / 6.0) * (k1x + 2*k2x + 2*k3x + k4x);
    y += (dt / 6.0) * (k1y + 2*k2y + 2*k3y + k4y);
    theta += (dt / 6.0) * (k1theta + 2*k2theta + 2*k3theta + k4theta);
    theta = std::atan2(std::sin(theta), std::cos(theta)); // Normalizzazione angolo

    //per correzzione con filtro gps
    if (gps_received_) {
    	double alpha = 0.05; // fusione lenta
    	x = (1.0 - alpha) * x + alpha * gps_position_.x;
    	y = (1.0 - alpha) * y + alpha * gps_position_.y;
	}


    double omega = (v / L) * std::tan(delta);

        // Validazione stato
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) {
            ROS_ERROR("Coordinate non finite! x=%.3f, y=%.3f, theta=%.3f", x, y, theta);
            x = 0;
            y = 0;
            theta = M_PI / 2.0;
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        //Odometry message
        nav_msgs::Odometry msg;
        msg.header.stamp = now;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "vehicle";

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        //Pose covariance (ho modificato solo i valori principali)
        std::fill(std::begin(msg.pose.covariance), std::end(msg.pose.covariance), 0.0);
        msg.pose.covariance[0] = 0.05;    // x
        msg.pose.covariance[7] = 0.05;    // y
        msg.pose.covariance[35] = 0.1;    // yaw

        msg.twist.twist.linear.x = v;
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = omega;

        // Twist covariance
        std::fill(std::begin(msg.twist.covariance), std::end(msg.twist.covariance), 0.0);
        msg.twist.covariance[0] = 0.1;    // linear.x
        msg.twist.covariance[35] = 0.2;   // angular.z

        pub_.publish(msg);

        // TF broadcast
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        q.setRPY(0, 0, theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, now, "odom", "vehicle"));

        ros::spinOnce();
        rate.sleep();
    }
}


private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::PointStamped current_value_;  // Valore corrente da pubblicare
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    ros::Time last_time_;
    double x, y, theta;
    bool received_first_msg_;

    //varibili per settaggio filtro di correzzione gps
    ros::Subscriber gps_sub_;
    geometry_msgs::Point gps_position_;
    bool gps_received_ = false;



    // Funzione callback che viene chiamata ogni volta che arriva un nuovo messaggio su /speedsteer
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        current_value_ = *msg;
        received_first_msg_ = true;
    }

    //funzione callback per filtro gps
    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg) {
   		 gps_position_ = msg->pose.pose.position;
    	gps_received_ = true;
    }

};

int main(int argc, char** argv) {
    ROS_INFO("odometer node started");
    ros::init(argc, argv, "odometer_node");
    OdometerNode node;
    node.spin();
    return 0;
}
