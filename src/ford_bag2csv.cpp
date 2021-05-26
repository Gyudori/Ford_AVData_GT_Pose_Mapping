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

std::queue<geometry_msgs::PoseStampedConstPtr> poses;
pcl::PointCloud<pcl::PointXYZI> laserCloud;

Eigen::Quaterniond q_lidar2vehicle(0.003582, -0.704648, -0.709546, 0.001830); // w, x, y, z
Eigen::Vector3d t_lidar2vehicle(1.13216, -0.378115, -1.40641);

std::fstream poseFs;
std::fstream scanFs;
std::fstream lidarFrameFs;

std::string INPUT_BAG_FILE_NAME;
std::string poseFileName = "/home/gyuseok/catkin_ws_kitti/result_data/ford_pose_gt.csv"; 
std::string scanFileName = "/home/gyuseok/catkin_ws_kitti/result_data/ford_scan_cloud.csv"; 
std::string lidarFrameFileName = "/home/gyuseok/catkin_ws_kitti/result_data/ford_scan_lidar_frame.csv";    

int frameCount = 0;

void writeScanPoints(pcl::PointCloud<pcl::PointXYZI> laserCloud){
    for(int i = 0; i < laserCloud.size(); i++){
                
        pcl::PointXYZI point = laserCloud.points[i];
        scanFs << frameCount << ",";
        scanFs << i << ",";         //Point Index
        scanFs << point.x << ",";
        scanFs << point.y << ",";
        scanFs << point.z << ",";
        scanFs << point.intensity << "\n";

        
        float distance = sqrt(pow(point.x, 2.0f) + pow(point.y, 2.0f) + pow(point.z, 2.0f));
        float vertAngle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        float horiAngle = atan2(point.y, point.x) * 180.0f / M_PI; ; // x축 기준 반시계 방향
        if(horiAngle < 0){
            horiAngle += 360;
        }

        lidarFrameFs << frameCount << ",";
        lidarFrameFs << i << ",";
        lidarFrameFs << distance << ",";
        lidarFrameFs << horiAngle << ",";
        lidarFrameFs << vertAngle << ",";
        lidarFrameFs << point.intensity << "\n";
    }
}

void writeLidarPoses(){
    for(int i = 0; i < 20; i++){    

        Eigen::Quaterniond q_vehicle2world(poses.front()->pose.orientation.w, poses.front()->pose.orientation.x, poses.front()->pose.orientation.y, poses.front()->pose.orientation.z);
        Eigen::Vector3d t_vehicle2world(poses.front()->pose.position.x, poses.front()->pose.position.y, poses.front()->pose.position.z);

        Eigen::Quaterniond q_lidarWorldPose;
        Eigen::Vector3d t_lidarWorldPose;

        q_lidarWorldPose = q_vehicle2world * q_lidar2vehicle;
        q_lidarWorldPose = q_lidarWorldPose.inverse();
        t_lidarWorldPose = q_vehicle2world * t_lidar2vehicle + t_vehicle2world;

        poseFs << t_lidarWorldPose.x() << ", ";
        poseFs << t_lidarWorldPose.y() << ", ";
        poseFs << t_lidarWorldPose.z() << ", ";
        poseFs << q_lidarWorldPose.x() << ", ";
        poseFs << q_lidarWorldPose.y() << ", ";
        poseFs << q_lidarWorldPose.z() << ", ";
        poseFs << q_lidarWorldPose.w() << ", ";

        if(i == 0){
            poseFs << frameCount << ", ";
            float startOri = atan2(laserCloud.points[0].y, laserCloud.points[0].x);
            startOri = startOri * 180.0f / M_PI;  
            if(startOri < 0){
                startOri += 360;
            }
            poseFs << startOri << "\n";            
        }
        else{
            poseFs << frameCount << "\n";
        }        
        if(poses.size() > 1){
            poses.pop();
        }
    }
}


void cloudInt2ptIdx(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudPtr){

    for(size_t i = 0; i < laserCloudPtr->points.size(); i ++){
        laserCloudPtr->points[i].intensity = (float)i;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_bag2csv");
    ros::NodeHandle nh;
    nh.getParam("bag_file_name", INPUT_BAG_FILE_NAME);

    rosbag::Bag bag;
    bag.open(INPUT_BAG_FILE_NAME, rosbag::BagMode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/pose_ground_truth"));
    topics.push_back(std::string("/velodyne_points"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));  
             
    poseFs.open(poseFileName, std::fstream::out);             
    scanFs.open(scanFileName, std::fstream::out);           
    lidarFrameFs.open(lidarFrameFileName, std::fstream::out); 

    // 시간의 순서에 따라 입력되는 topic을 확인한다. 맨 첫 라이다 스캔은 포즈보다 빨리 들어오므로
    // 첫 포즈부터 순차적으로 poses에 추가되고, scan이 새롭게 입력되는 시점에 poses 연산을 수행한다.
    foreach(rosbag::MessageInstance const m, view){
        geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
        sensor_msgs::PointCloud2ConstPtr scan = m.instantiate<sensor_msgs::PointCloud2>();
        
        if(scan != NULL){ 

            if(!poses.empty()){
                writeLidarPoses();                
                std::queue<geometry_msgs::PoseStampedConstPtr> empty;
                std::swap(poses, empty);
            }  

            pcl::fromROSMsg(*scan, laserCloud);     
            writeScanPoints(laserCloud);
            
            if(frameCount == 0){
                pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*scan, *laserCloudPtr);

                cloudInt2ptIdx(laserCloudPtr);                
                
                pcl::PLYWriter writer;
                writer.write<pcl::PointXYZI>("/home/gyuseok/catkin_ws_kitti/result_data/onescan.ply", *laserCloudPtr);
            }
            std::cout << "Current Frame: " << frameCount << "\n";
            frameCount++;
            
        }           
       
        if(pose != NULL){
            poses.push(pose);           
        }
    }

    std::cout << "Start writing CSV files... \n";
    poseFs.close();
    scanFs.close();
    lidarFrameFs.close();
    bag.close();

    std::cout << "Done...!\n";
}


