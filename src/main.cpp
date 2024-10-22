#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ObstacleDetector {
public:
    ObstacleDetector(ros::NodeHandle &nh, const std::string &scan_topic, float distance_threshold) 
        : obstacle_detected_(false), distance_threshold_(distance_threshold) {
        scan_sub_ = nh.subscribe(scan_topic, 10, &ObstacleDetector::scanCallback, this);
    }

    bool isObstacleDetected() const {
        return obstacle_detected_;
    }

private:
    ros::Subscriber scan_sub_;
    bool obstacle_detected_;
    float distance_threshold_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        obstacle_detected_ = false;
        for (const float &range : msg->ranges) {
            if (range < distance_threshold_) {
                obstacle_detected_ = true;
                ROS_WARN("Obstacle detected at %.2f meters!", range);
                return;
            }
        }
    }
};

class PathFollower {
public:
    PathFollower(ros::NodeHandle &nh, const std::string &path_topic, const std::string &cmd_vel_topic)
        : nh_(nh) {
        path_sub_ = nh.subscribe(path_topic, 10, &PathFollower::pathCallback, this);
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    }

    void followPath(bool obstacle_detected) {
        geometry_msgs::Twist cmd_vel;
        if (!obstacle_detected && !current_path_.poses.empty()) {
            cmd_vel.linear.x = 0.5;  
            cmd_vel.angular.z = 0.0; 
        } else {
            stopRobot();
        }
        cmd_pub_.publish(cmd_vel);
    }

    void stopRobot() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        cmd_pub_.publish(stop_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    nav_msgs::Path current_path_;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        current_path_ = *msg;
        ROS_INFO("Path received.");
    }
};

class MapManager {
public:
    MapManager(ros::NodeHandle &nh, const std::string &map_service_name, const std::string &initial_map_path)
        : nh_(nh), map_service_name_(map_service_name), initial_map_path_(initial_map_path) {
        map_service_ = nh_.advertiseService(map_service_name_, &MapManager::switchMap, this);
        loadPGMMap(initial_map_path_, current_map_);
    }

    bool loadPGMMap(const std::string &pgm_file, nav_msgs::OccupancyGrid &map) {
        cv::Mat image = cv::imread(pgm_file, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            ROS_ERROR("Failed to load PGM map.");
            return false;
        }

        int width = image.cols;
        int height = image.rows;
        map.info.width = width;
        map.info.height = height;
        map.info.resolution = 0.05;  
        map.data.resize(width * height);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int pixel = image.at<uchar>(i, j);
                map.data[i * width + j] = (pixel == 255) ? 0 : 100;  
            }
        }
        ROS_INFO("PGM map loaded successfully.");
        return true;
    }

    bool switchMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Switching maps...");
        std::string new_map_path = "~/ws/src/test/maps/test_map.pgm";  
        if (!loadPGMMap(new_map_path, current_map_)) {
            ROS_ERROR("Failed to switch maps.");
            return false;
        }
        ROS_INFO("Map switched successfully.");
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer map_service_;
    std::string map_service_name_;
    std::string initial_map_path_;
    nav_msgs::OccupancyGrid current_map_;
};

class RobotSimulation {
public:
    RobotSimulation(ros::NodeHandle &nh) 
        : obstacle_detector_(nh, "scan", 0.5),
          path_follower_(nh, "path", "cmd_vel"),
          map_manager_(nh, "switch_map", "~/ws/src/test/maps/blank_map_with_obstacle.pgm") { 
    }

    void run() {
        ros::Rate rate(10);  
        while (ros::ok()) {
            path_follower_.followPath(obstacle_detector_.isObstacleDetected());
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ObstacleDetector obstacle_detector_;
    PathFollower path_follower_;
    MapManager map_manager_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_simulation");
    ros::NodeHandle nh;

    RobotSimulation robot_simulation(nh);
    robot_simulation.run();

    return 0;
}
