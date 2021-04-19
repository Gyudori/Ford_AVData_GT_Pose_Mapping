#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <ford_mapping_lib/PointXYZIRD.hpp>

#include <mutex>
#include <queue>
#include <cmath>
#include <iostream>
#include <thread>
#include <sstream>
#include <fstream>

std::mutex mBuf;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtOneScanBuf;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<PointXYZIRD>::Ptr laserCloudIR(new pcl::PointCloud<PointXYZIRD>());

pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<PointXYZIRD>::Ptr lidarMapIR(new pcl::PointCloud<PointXYZIRD>());

geometry_msgs::PoseStamped::Ptr poseGt(new geometry_msgs::PoseStamped());

uint64_t poseTime;
uint64_t currLaserCloudTime;
uint64_t preLaserCloudTime = 0;

int waitingMsec = 0;
int frameCount = 0;
int LOAD_FRAME_NUM;
int SKIP_FRAME_NUM;
bool printedFlag = false;
bool startedFlag = false;
bool endFlag = false;

Eigen::Quaterniond q_lidarBody(0.003582, -0.704648, -0.709546, 0.001830); // w, x, y, z
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
    Eigen::Vector3d point_w = q_lidarBody * point_curr + t_lidarBody;
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void pointTransform(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po,
Eigen::Quaterniond q, Eigen::Vector3d t)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q * point_curr + t;
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void systemCalibration(PointXYZIRD const *const pi, PointXYZIRD *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_lidarBody * point_curr + t_lidarBody;
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
    po->ring = pi->ring;
    po->distance = pi->distance;
}

void pointTransform(PointXYZIRD const *const pi, PointXYZIRD *const po,
Eigen::Quaterniond q, Eigen::Vector3d t)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q * point_curr + t;
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
    po->ring = pi->ring;
    po->distance = pi->distance;
}

void pointOrganize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, pcl::PointCloud<PointXYZIRD>::Ptr cloudOut){
    
    for(size_t i = 0; i < cloudIn->points.size(); i++){
        pcl::PointXYZI pointIn(cloudIn->points[i]);

        float verticalAngle = atan(pointIn.z / sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y)) * 180 / M_PI;
        int ring = int((verticalAngle + 92.0/3.0) * 3.0 / 4.0 + 0.5);       
        
        float distance = sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y + pointIn.z * pointIn.z);      

        PointXYZIRD pointOut;

        pointOut.x = pointIn.x;
        pointOut.y = pointIn.y;
        pointOut.z = pointIn.z;
        pointOut.intensity = pointIn.intensity;
        pointOut.ring = ring;
        pointOut.distance = distance;

        // // Debug
        // if(i % (cloudIn->points.size() / 100) == 0){
        //     std::cout << "Ring Num: " << ring << "\n";
        // }        

        cloudOut->points.push_back(pointOut);
    }    
    
}

template<typename T>
void waitForEmptyQueue(std::queue<T> &queueBuf){
    while(queueBuf.empty()){           
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);  
        waitingMsec +=10;

        if(waitingMsec > 1000){
            break;
        }      
    }
}

void process(){
    while(1){

        if(endFlag){
            break;
        }

        std::cout << "Process running... \n";
        while(!laserCloudBuf.empty() && !poseGtBuf.empty()){           
            
            startedFlag = true;
            waitingMsec = 0;

            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();            
           
            laserCloud->clear(); 
            
            pcl::fromROSMsg(*laserCloudBuf.front(), *laserCloud);
            laserCloudBuf.pop();            

            laserCloudIR->clear();
            pointOrganize(laserCloud, laserCloudIR);

            waitForEmptyQueue(laserCloudBuf);             
            preLaserCloudTime = currLaserCloudTime;
            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();    

            
            // Starting with Pose            
            while(poseGtBuf.front()->header.stamp.toNSec() < preLaserCloudTime && !poseGtBuf.empty()){
                poseGtBuf.pop();
            } 

            waitForEmptyQueue(poseGtBuf);
            while(poseGtBuf.front()->header.stamp.toNSec() >= preLaserCloudTime &&
            poseGtBuf.front()->header.stamp.toNSec() < currLaserCloudTime)
            {                   
                // std::cout << "PoseGT Time: " << poseGtBuf.front()->header.stamp.toNSec() << "\n";
                                
                poseGtOneScanBuf.push(poseGtBuf.front());
                poseGtBuf.pop();  
                waitForEmptyQueue(poseGtBuf);                           
            }

            // 1개 스캔의 포즈들에 따라 점군 매핑
            size_t count = 0;
            while(!poseGtOneScanBuf.empty()){

                Eigen::Quaterniond q_curr;
                Eigen::Vector3d t_curr;
                poseTime = poseGtOneScanBuf.front()->header.stamp.toNSec();
                
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
                size_t pointNumEnd = (size_t)(poseTimeScanning / laserCloudScanningTime) * (size_t)laserCloudIR->points.size();
                if(poseGtOneScanBuf.empty()){
                    pointNumEnd = laserCloudIR->points.size();
                }
                for(size_t i = count; i < pointNumEnd; i++){                      
                    systemCalibration(&laserCloudIR->points[i], &laserCloudIR->points[i]);
                    pointTransform(&laserCloudIR->points[i], &laserCloudIR->points[i], q_curr, t_curr);               
                    
                    
                    if(frameCount % SKIP_FRAME_NUM == 0){
                        if(!printedFlag){
                            std::cout << "Frame: " << frameCount << "\n";
                            std::cout << "Curr x: " << t_curr.x() << "\nCurr y: " << t_curr.y() << "\nCurr z: " << t_curr.z() << "\n";
                            printedFlag = true;
                        }                        
                        // lidarMap->points.push_back(laserCloud->points[i]);
                        lidarMapIR->points.push_back(laserCloudIR->points[i]);
                    }                    
                }                
                count = pointNumEnd;    
            } 
            
            printedFlag = false;

            // Write PLY file in Loop
            // if(frameCount == LOAD_FRAME_NUM){
            //     std::cout << "End of stream \n Ply file writing";
            //     pcl::PLYWriter writer;    
            //     writer.write<pcl::PointXYZI> 
            //         ("/home/gyuseok/catkin_ws_kitti/result_data/ford_mapping_test.ply"
            //         ,*lidarMap, false);
            // }
            
            std::cout << "Current frame count: " << frameCount << "\n";
            frameCount++;    
        }

        while((laserCloudBuf.empty() || poseGtBuf.empty()) && startedFlag){
            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
            waitingMsec += 100;
            
            if(waitingMsec > 3000){
                std::cout << "End of stream\nWriting ply file... \n";
                pcl::PLYWriter writer;    
                writer.write<PointXYZIRD> 
                    ("/home/gyuseok/catkin_ws_kitti/result_data/ford_mapping_test.ply"
                    ,*lidarMapIR, false);
                std::cout << "Ply writing done! \n Writing txt file... \n";

                std::fstream fs;                
                fs.open("/home/gyuseok/catkin_ws_kitti/result_data/ford_mapping_test.txt", std::fstream::out);
                for(size_t i = 0; i < lidarMapIR->points.size(); i++){
                    fs << lidarMapIR->points[i].x << " ";
                    fs << lidarMapIR->points[i].y << " ";
                    fs << lidarMapIR->points[i].z << " ";
                    fs << lidarMapIR->points[i].intensity << " ";
                    fs << lidarMapIR->points[i].ring << " ";
                    fs << lidarMapIR->points[i].distance << "\n";                    
                }
                fs.close();
                std::cout << "Txt writing done! \n";
                endFlag = true;
                break;
            }
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