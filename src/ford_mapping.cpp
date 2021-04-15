#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <mutex>
#include <queue>
#include <cmath>
#include <iostream>
#include <thread>

std::mutex mBuf;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtOneScanBuf;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMap(new pcl::PointCloud<pcl::PointXYZI>());
geometry_msgs::PoseStamped::Ptr poseGt(new geometry_msgs::PoseStamped());

uint64_t poseTime;
uint64_t currLaserCloudTime;
uint64_t preLaserCloudTime = 0;

int scanCount = 0;
int LOAD_FRAME_NUM;
int SKIP_FRAME_NUM;

Eigen::Quaterniond q_lidarBody(0.003582, -0.704648, -0.709546, 0.001830);
Eigen::Vector3d t_lidarBody(1.13216, -0.378115, -1.40641);

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    mBuf.lock();
    laserCloudBuf.push(laserCloudMsg);
    mBuf.unlock();
}

void poseGtHandler(const geometry_msgs::PoseStampedConstPtr &poseGtMsg){
    mBuf.lock();
    poseGtBuf.push(poseGtMsg);
    mBuf.unlock();
}

void systemCalibration(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_lidarBody * (point_curr + t_lidarBody);
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void pointTransform(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po,
Eigen::Quaterniond q, Eigen::Vector3d t)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q * (point_curr + t);
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void process(){
    while(1){
        std::cout << "Process running... \n";
        while(!laserCloudBuf.empty() && !poseGtBuf.empty()){
            // Need time sync here

            // laserCloud header timestamp는 0.1초 스캐닝을 시작할 때의 시각
            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();

            laserCloud->clear(); 
            pcl::fromROSMsg(*laserCloudBuf.front(), *laserCloud);
            laserCloudBuf.pop();

            while(laserCloudBuf.empty()){
                std::chrono::milliseconds dura(100);
                std::this_thread::sleep_for(dura);
            }

            preLaserCloudTime = currLaserCloudTime;
            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();
            std::cout << "PrelaserCloudTime: " << preLaserCloudTime << "\n";
            std::cout << "CurlaserCloudTime: " << currLaserCloudTime << "\n";


            while(poseGtBuf.front()->header.stamp.toNSec() < preLaserCloudTime && !poseGtBuf.empty()){
                poseGtBuf.pop();
            } 

            // 1개의 scan 내에 포함되는 pose queue 생성
            while(poseGtBuf.front()->header.stamp.toNSec() >= preLaserCloudTime &&
            poseGtBuf.front()->header.stamp.toNSec() < currLaserCloudTime)
            {   
                std::cout << "PoseGT Time: " << poseGtBuf.front()->header.stamp.toNSec() << "\n";
                if(poseGtBuf.empty()){
                    std::chrono::milliseconds dura(10);
                    std::this_thread::sleep_for(dura);
                }
                else{
                    poseGtOneScanBuf.push(poseGtBuf.front());
                    poseGtBuf.pop();
                }                
            }
            // 1개 스캔의 포즈들에 따라 점군 매핑
            size_t count = 0;
            while(!poseGtOneScanBuf.empty()){

                poseTime = poseGtOneScanBuf.front()->header.stamp.toNSec();
                Eigen::Quaterniond q_curr;
                Eigen::Vector3d t_curr;
                q_curr.x() = poseGtOneScanBuf.front()->pose.orientation.x;
                q_curr.y() = poseGtOneScanBuf.front()->pose.orientation.y;
                q_curr.z() = poseGtOneScanBuf.front()->pose.orientation.z;
                q_curr.w() = poseGtOneScanBuf.front()->pose.orientation.w;
                t_curr.x() = poseGtOneScanBuf.front()->pose.position.x;
                t_curr.y() = poseGtOneScanBuf.front()->pose.position.y;
                t_curr.z() = poseGtOneScanBuf.front()->pose.position.z;
                poseGtOneScanBuf.pop();

                uint64_t laserCloudScanningTime = currLaserCloudTime - preLaserCloudTime;
                uint64_t poseTimeScanning = poseTime - preLaserCloudTime;
                size_t pointNumEnd = (size_t)(poseTimeScanning / laserCloudScanningTime) * (size_t)laserCloud->points.size();
                if(poseGtOneScanBuf.empty()){
                    pointNumEnd = laserCloud->points.size();
                }
                for(size_t i = count; i < pointNumEnd; i++){
                    systemCalibration(&laserCloud->points[i], &laserCloud->points[i]);
                    pointTransform(&laserCloud->points[i], &laserCloud->points[i], q_curr, t_curr);
                    
                    if(scanCount % SKIP_FRAME_NUM == 0){
                        lidarMap->points.push_back(laserCloud->points[i]);
                    }
                    
                }
                count = pointNumEnd;    
            } 

            if(scanCount == LOAD_FRAME_NUM){
                pcl::PLYWriter writer;    
                writer.write<pcl::PointXYZI> 
                    ("/home/gyuseok/catkin_ws_kitti/result_data/ford_mapping_test.ply"
                    ,*lidarMap, false);
            }
            scanCount++;
            
            // For Debug
            // pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIndexedPtr(new pcl::PointCloud<pcl::PointXYZI>());
            // float count = 0.0f;
            // for(size_t i = 0; i < laserCloud->points.size(); i++){
            //     pcl::PointXYZI point;
            //     point.x = laserCloud->points[i].x;
            //     point.y = laserCloud->points[i].y;
            //     point.z = laserCloud->points[i].z;
            //     point.intensity = count;

            //     laserCloudIndexedPtr->points.push_back(point);
            //     count++;
            // }            
            // pcl::PLYWriter writer;
            // writer.write<pcl::PointXYZI> ("/home/gyuseok/catkin_ws_kitti/result_data/ford_one_frame.ply",
            //                             *laserCloudIndexedPtr, false);


        }

        
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ford_mapping");
    ros::NodeHandle nh;

    nh.getParam("load_frame_num", LOAD_FRAME_NUM);
    nh.getParam("skip_frame_num", SKIP_FRAME_NUM);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                ("/velodyne_points", 100, laserCloudHandler);

    ros::Subscriber subGtPose = nh.subscribe<geometry_msgs::PoseStamped>
                                ("/pose_ground_truth", 100, poseGtHandler);
    
    std::thread mapping_process{process};

    ros::spin();

    return 0;
}