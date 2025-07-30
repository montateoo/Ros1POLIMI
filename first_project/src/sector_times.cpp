#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "first_project/sector_times.h"

class SectorTimesNode {
public:
    SectorTimesNode() {
        speed_sub_ = nh.subscribe("/speedsteer", 1, &SectorTimesNode::speedCallback, this);
        gps_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 1, &SectorTimesNode::gpsCallback, this);
        sector_pub_ = nh.advertise<first_project::sector_times>("/sector_times", 1);

        //setting default values
        current_sector_ = 0;
        sector_time_ = 0.0;
        mean_speed_ = 0.0;
        num_data_ = 0;

        //inizializzazione delle coordinate(circa) gps di entrata dei tre settori
        //una volta che so il settore di partenza il cambio avviene in modo sequenziale
        s2.latitude = 45.630115;
        s2.longitude = 9.290241;
        s2.header.frame_id = "sector2";

        s3.latitude = 45.623367;
        s3.longitude = 9.286977;
        s3.header.frame_id = "sector3";

        s1.latitude = 45.616178;
        s1.longitude = 9.280884;
        s1.header.frame_id = "sector1";
    }

    //dati tre punti traccio una rett passante per i primi due e controllo a che lato (sx o dx) appartiene il terzo punto
    //utilizzo il prodotto vettoriale per determinare da che settore parte la macchina
    bool initial_location(double ax, double ay, double bx, double by, double px, double py) {
    	double D = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    	if (D > 0) {
     	   return true;
   		} else{
       	   return false;
    	}
    }

    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
      float speed = msg->point.y;

      //considero l'avvio del nodo solo dopo che la macchina parte(velocità > 0)
      if(!received_first_data_){
        if(speed>0){
           received_first_data_ = true;
           sector_start_time_ = ros::Time::now();
          }
          else{
            return;
          }
      }

      num_data_++;
      if(num_data_ != 1) {
        mean_speed_ = (mean_speed_ * (num_data_ - 1) + speed)/num_data_;
      }else{
        mean_speed_ = speed;
      }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        //coordinate nulle o non valide vengono scartate
        if (msg->latitude == 0.0 && msg->longitude == 0.0) {
            //ROS_WARN("GPS data ignored: null coordinates [ %f , %f ]", msg->latitude, msg->longitude);
            return;
        }

        //appena ricevo il primo messaggio gps verifico in che settore mi trovo
        if(current_sector_ == 0) {
           if(initial_location(s1.longitude,s1.latitude,s2.longitude,s2.latitude,msg->longitude,msg->latitude)){
             current_sector_ = 1;
           }
           else if(!initial_location(s2.longitude,s2.latitude,s3.longitude,s3.latitude,msg->longitude,msg->latitude)){
             current_sector_ = 2;
           }else{
             current_sector_ = 3;
           }
           ROS_INFO("Car starting from sector: %d",current_sector_);
        }


        //passaggio da s1 a s2
        if(current_sector_ == 1) {
            if(msg->latitude > s2.latitude && msg->longitude > s2.longitude) {
                ros::Duration sector_duration = ros::Time::now() - sector_start_time_;
                ROS_INFO("sector changed s1->s2 [ %f , %f ] | time: %.2f s | mean speed: %.2f",
                         msg->latitude, msg->longitude, sector_duration.toSec(), mean_speed_);
                current_sector_++;
                mean_speed_ = 0;
                num_data_ = 0;
                sector_start_time_ = ros::Time::now();
            }
        }
        //passaggio da s2 a s3
        else if(current_sector_ == 2) {
            if(msg->latitude < s3.latitude && msg->longitude < s3.longitude) {
                ros::Duration sector_duration = ros::Time::now() - sector_start_time_;
                ROS_INFO("sector changed s2->s3 [ %f , %f ] | time: %.2f s | mean speed: %.2f",
                         msg->latitude, msg->longitude, sector_duration.toSec(), mean_speed_);
                current_sector_++;
                mean_speed_ = 0;
                num_data_ = 0;
                sector_start_time_ = ros::Time::now();
            }
        }
        //passaggio da s3 a s1
        else if(current_sector_ == 3) {
            if(msg->latitude > s1.latitude && msg->longitude < s1.longitude) {
                ros::Duration sector_duration = ros::Time::now() - sector_start_time_;
                ROS_INFO("sector changed s3->s1 [ %f , %f ] | time: %.2f s | mean speed: %.2f",
                         msg->latitude, msg->longitude, sector_duration.toSec(), mean_speed_);
                current_sector_ = 1;
                mean_speed_ = 0;
                num_data_ = 0;
                sector_start_time_ = ros::Time::now();
            }
        }
        else {
            ROS_ERROR("SectorTimesNode Error");
        }
    }

    void spin() {
    	ros::Rate rate(10);
   		while (ros::ok()) {
          ros::spinOnce();

          if (received_first_data_) {
            ros::Duration sector_duration = ros::Time::now() - sector_start_time_;
            sector_time_ = sector_duration.toSec();

            first_project::sector_times msg;
            msg.current_sector = current_sector_;
            msg.current_sector_time = sector_time_;
            msg.current_sector_mean_speed = mean_speed_;
            sector_pub_.publish(msg);
          }

          rate.sleep();
    }
}



private:
    ros::NodeHandle nh;

    ros::Subscriber speed_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher sector_pub_;

    sensor_msgs::NavSatFix s1,s2,s3;
    ros::Time sector_start_time_;
    float sector_time_,mean_speed_;
    int num_data_,current_sector_;
    bool received_first_data_;

};

int main(int argc, char** argv) {
    ROS_INFO("sector_times node started");
    ros::init(argc, argv, "sector_times");
    SectorTimesNode node;
    node.spin();
    return 0;
}

