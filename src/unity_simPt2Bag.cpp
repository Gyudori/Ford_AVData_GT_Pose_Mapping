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
    ros::init(argc, argv, "unity_simPt2Bag");
    ros::Time::init();

    ros::NodeHandle nh;
    std::string INPUT_FILE_NAME, OUTPUT_BAG_FILE;
    nh.getParam("input_file_name", INPUT_FILE_NAME);
    nh.getParam("output_bag_file", OUTPUT_BAG_FILE);    

    rosbag::Bag bag;
    bag.open(OUTPUT_BAG_FILE, rosbag::BagMode::Write);    
         
    std::ifstream inputFile(INPUT_FILE_NAME, std::ifstream::in);

    std::string line, s;
    int frameCount = 1, currFrameCount = 1;
    
    std::getline(inputFile, line); // Frist line is title

    for(int j = 0; j < 80; j++){
        
        pcl::PointCloud<pcl::PointXYZI> laserCloud;              
        
        for(int i = 0; i < 57600; i++){
            std::getline(inputFile, line);
            std::stringstream pointStream(line);
            pcl::PointXYZI point;
            
            std::getline(pointStream, s, ','); 
            currFrameCount = std::stoi(s);

            std::getline(pointStream, s, ','); // Remove timestamp

            std::getline(pointStream, s, ',');
            point.x = std::stof(s);

            std::getline(pointStream, s, ',');
            point.y = std::stof(s);

            std::getline(pointStream, s, ',');
            point.z = std::stof(s);

            std::getline(pointStream, s, ',');
            float intensity = std::stof(s);

            if(point.x != .0f && point.y != .0f && point.z != .0f){
                laserCloud.push_back(point);
            }
            else{
                // std::cout << "zero point\n";
            }                
        }

        frameCount = currFrameCount;
        sensor_msgs::PointCloud2 laserCloudMsg;
        pcl::toROSMsg(laserCloud, laserCloudMsg);
        laserCloudMsg.header.frame_id = "/camera_init";

        bag.write("/velodyne_points", ros::Time::now(), laserCloudMsg);
        std::cout << "Frame: " << frameCount - 1 << "  CloudSize: " << laserCloud.size() << "\n";
    }

    bag.close(); 

    return 0;    
}


