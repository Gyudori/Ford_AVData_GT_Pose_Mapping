#include <sstream>
#include <fstream>
#include <iostream>
#include <queue>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define foreach BOOST_FOREACH

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_bag2csv");
    rosbag::Bag bag;
    bag.open("/home/gyuseok/catkin_ws_kitti/result_data/ford_mapping_odometry.bag",
    rosbag::BagMode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/pose_ground_truth"));
    topics.push_back(std::string("/velodyne_points"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // std::fstream fs;      
    // std::string file_name = "/home/gyuseok/catkin_ws_kitti/result_data/ford_pose_gt.csv";          
    // fs.open(file_name, std::fstream::out);

    int frameCount = -1;
    int poseCount = 0;
    uint32_t scanTime;
    uint32_t poseTime;

    std::queue<geometry_msgs::PoseStampedConstPtr> poses;

    std::fstream fs;
    std::string file_name = "/home/gyuseok/catkin_ws_kitti/result_data/ford_pose_gt.csv";          
    fs.open(file_name, std::fstream::out); 

    foreach(rosbag::MessageInstance const m, view){
        geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
        sensor_msgs::PointCloud2ConstPtr scan = m.instantiate<sensor_msgs::PointCloud2>();
        
        if(scan != NULL){
            frameCount++; 
            if(!poses.empty()){
                for(int i = 0; i < 20; i++){
                    fs << poses.front()->pose.position.x << ", ";
                    fs << poses.front()->pose.position.y << ", ";
                    fs << poses.front()->pose.position.z << ", ";
                    fs << poses.front()->pose.orientation.x << ", ";
                    fs << poses.front()->pose.orientation.y << ", ";
                    fs << poses.front()->pose.orientation.z << ", ";
                    fs << poses.front()->pose.orientation.w << ", ";
                    fs << frameCount << "\n";
                    
                    if(poses.size() > 1){
                        poses.pop();
                    }
                }
                std::queue<geometry_msgs::PoseStampedConstPtr> empty;
                std::swap(poses, empty);
            }

            if(frameCount == 0){
                pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*scan, *laserCloud);
                pcl::PLYWriter writer;
                writer.write<pcl::PointXYZI>("/home/gyuseok/catkin_ws_kitti/result_data/onescan.ply", *laserCloud);
            }
        }           
       
        if(pose != NULL){
            poses.push(pose);           
        }
    }
    fs.close();

    bag.close();
}


