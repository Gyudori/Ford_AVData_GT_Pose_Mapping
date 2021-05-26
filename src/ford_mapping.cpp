#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>

#include <ford_mapping_lib/PointXYZIRD.hpp>

#include <mutex>
#include <queue>
#include <cmath>
#include <iostream>
#include <thread>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>


std::mutex mBuf;

rosbag::Bag bag_out;

std::queue<sensor_msgs::PointCloud2ConstPtr> laserCloudBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> poseGtOneScanBuf;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<PointXYZIRD>::Ptr laserCloudIR(new pcl::PointCloud<PointXYZIRD>());

pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<PointXYZIRD>::Ptr lidarMapIR(new pcl::PointCloud<PointXYZIRD>());
pcl::PointCloud<PointXYZIRD>::Ptr lidarMapIRVoxelFiltered(new pcl::PointCloud<PointXYZIRD>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarMapRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr lidarMapRGBNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarViewPoint(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::Normal>::Ptr lidarNormals(new pcl::PointCloud<pcl::Normal>());

geometry_msgs::PoseStamped::Ptr poseGt(new geometry_msgs::PoseStamped());

uint64_t poseTime;
uint64_t currLaserCloudTime;
uint64_t preLaserCloudTime = 0;

std::vector<int> scanEndInd;

int waitingMsec = 0;
int frameCount = 0;
int LOAD_FRAME_NUM;
int SKIP_FRAME_NUM;
double DIST_THRES = 100, LEAF_SIZE = 0.1;
std::string OUTPUT_BAG_FILE, OUTPUT_PLY_FILE;
bool TO_BAG = false, TO_MESHLAB_PLY = false;
bool MOTION_CORRECTED = true;
bool printedFlag = false;
bool startedFlag = false;
bool endFlag = false;
bool oneFrameMappingDone = false;

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

// TO DO: Integrate functions of system calibration and pose transform 
void systemCalibration(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_lidarBody * point_curr + t_lidarBody;
    po->x = point_w.x();
    po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void pointTransform(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po, Eigen::Quaterniond q, Eigen::Vector3d t)
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

void pointTransform(PointXYZIRD const *const pi, PointXYZIRD *const po, Eigen::Quaterniond q, Eigen::Vector3d t)
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

pcl::PointCloud<PointXYZIRD>::Ptr removeOverOneFrame(pcl::PointCloud<PointXYZIRD>::Ptr cloudIn)
{
    pcl::PointCloud<PointXYZIRD>::Ptr cloudOut(new pcl::PointCloud<PointXYZIRD>());
    float startOri = atan2(cloudIn->points[0].y, cloudIn->points[0].x);

    if(startOri < 0){
        startOri = -startOri;
    }
    else{
        startOri = 2 * M_PI - startOri;
    }

    // For debug
    if(frameCount == 19){
        int a = 1;
    }

    float endOri = startOri + 2 * M_PI;
    int poseInd = 1;
    bool exceedAxis = false;

    for(int i = 0; i < cloudIn->points.size(); i++){
        float pointOri = atan2(cloudIn->points[i].y, cloudIn->points[i].x);
        
        // Orientation unify
        if(pointOri < 0){
            pointOri = -pointOri;
        }
        else{
            pointOri = 2 * M_PI - pointOri;
        }

        // Make orientation incremental
        if(pointOri < startOri - 0.2f * M_PI / 180){
             exceedAxis = true;
        }

        if(exceedAxis){
            pointOri += 2 * M_PI;
        }

        // For debug
        // cloudIn->points[i].distance = pointOri;    

        // Indexing mapping point range per pose
        if(pointOri >= startOri + 2 * M_PI/poseGtOneScanBuf.size() * poseInd){
            scanEndInd.push_back(i-1);
            poseInd++;
        }    

        // Distinguish orientation larger than one frame
        if(pointOri >= endOri){
             break;
        }
        else{
            cloudOut->points.push_back(cloudIn->points[i]);
        }         
    }

    return cloudOut;
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
        pointOut.ring = frameCount;
        pointOut.distance = distance;  
        // pointOut.distance = (float)frameCount;    

        if(distance < DIST_THRES){
            cloudOut->points.push_back(pointOut);
        }
        
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

void writeRgbMapForMeshLab(pcl::PointCloud<PointXYZIRD>::Ptr lidarMap){

    for(size_t i = 0; i < lidarMap->points.size(); i++){
        
        pcl::PointXYZRGBNormal point;
        point.x = lidarMap->points[i].x;
        point.y = lidarMap->points[i].y;
        point.z = lidarMap->points[i].z;
        point.r = (uint8_t)lidarMap->points[i].intensity;
        point.g = (uint8_t)lidarMap->points[i].intensity;
        point.b = (uint8_t)lidarMap->points[i].intensity;
        point.normal_x = lidarNormals->points[i].normal_x;
        point.normal_y = lidarNormals->points[i].normal_y;
        point.normal_z = lidarNormals->points[i].normal_z;
        lidarMapRGBNormal->points.push_back(point);              
    }  
    pcl::io::savePLYFileBinary(OUTPUT_PLY_FILE+"vfiltered_rgb.ply",*lidarMapRGBNormal);   
    std::cout << "XYZRGBNormal Ply writing done!";
}

void computeNormal(pcl::PointCloud<PointXYZIRD>::Ptr lidarMap){   

    float currFrame = 0;
    int j = 0;  
    
    for(int i = 0; i < lidarViewPoint->points.size(); i++){              
        std::vector<int> indices;
        while(lidarMap->points[j].ring == currFrame){ // ring is frameCount
            indices.push_back(j);
            j++;
            if(lidarMap->points.size() == j){
                break;
            }
        }
        currFrame++;

        pcl::NormalEstimation<PointXYZIRD, pcl::Normal> ne;
        ne.setInputCloud(lidarMap);

        boost::shared_ptr<std::vector<int>> indicesPtr(new std::vector<int>(indices));
        ne.setIndices(indicesPtr);
        
        pcl::search::KdTree<PointXYZIRD>::Ptr tree (new pcl::search::KdTree<PointXYZIRD>());
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        ne.setKSearch(10);

        ne.compute(*cloud_normals);

        for(int k = 0; k < indices.size(); k++){

            float &nx = cloud_normals->points[k].normal_x;
            float &ny = cloud_normals->points[k].normal_y;
            float &nz = cloud_normals->points[k].normal_z;

            pcl::flipNormalTowardsViewpoint(lidarMap->points[indices[k]], 
            lidarViewPoint->points[i].x, lidarViewPoint->points[i].y, lidarViewPoint->points[i].z,
            nx, ny, nz);

            cloud_normals->points[k].normal_x = nx;
            cloud_normals->points[k].normal_y = ny;
            cloud_normals->points[k].normal_z = nz;
        }

        *lidarNormals += *cloud_normals;
    }    
    std::cout << "Normal size: " << lidarNormals->points.size() << "\n";
    std::cout << "LidarCloud size: " << lidarMap->points.size() << "\n";
}

pcl::PointCloud<PointXYZIRD>::Ptr voxelFiltering(pcl::PointCloud<PointXYZIRD>::Ptr lidarMap){

    pcl::PointCloud<PointXYZIRD>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRD>());
    float leafSize = LEAF_SIZE;

    pcl::VoxelGrid<PointXYZIRD> vg;
    vg.setInputCloud(lidarMap);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*cloud_filtered);

    return cloud_filtered;
}

void writeTxtForCloudCompare(pcl::PointCloud<PointXYZIRD>::Ptr lidarMapIR, std::string fileName){

    std::fstream fs;                
    std::string fileDir = "/home/gyuseok/catkin_ws_kitti/result_data/" + fileName;
    fs.open(fileDir, std::fstream::out);
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
}


void process(){
    while(1){

        if(endFlag){
            break;
        }

        std::cout << "Process running... \n";
        while(!laserCloudBuf.empty()){
            
            startedFlag = true;
            waitingMsec = 0;

            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();  
            ros::Time currLaserStamp = laserCloudBuf.front()->header.stamp;
           
            laserCloud->clear(); 
            
            pcl::fromROSMsg(*laserCloudBuf.front(), *laserCloud);
            laserCloudBuf.pop();  

            std::cout << "Frame: " << frameCount << "\n";
            std::cout << "Before Cloud size: " << laserCloud->points.size() << "\n";          

            laserCloudIR->clear();
            pointOrganize(laserCloud, laserCloudIR);
            std::cout << "After Organize Cloud size: " << laserCloudIR->points.size() << "\n"; 

            waitForEmptyQueue(laserCloudBuf);   
            if(laserCloudBuf.empty()){ // 여기 손봐야됨. 일단 마지막꺼는 매핑 안하려고 이렇게 함.
                break;
            }      
            preLaserCloudTime = currLaserCloudTime;
            currLaserCloudTime = laserCloudBuf.front()->header.stamp.toNSec();   

            
            // Delete pose before the start time of laser scan
            while(poseGtBuf.front()->header.stamp.toNSec() < preLaserCloudTime && !poseGtBuf.empty()){
                poseGtBuf.pop();
            } 

            waitForEmptyQueue(poseGtBuf);
            while(!poseGtBuf.empty() && poseGtBuf.front()->header.stamp.toNSec() >= preLaserCloudTime &&
            poseGtBuf.front()->header.stamp.toNSec() < currLaserCloudTime)
            {
                // std::cout << "PoseGT Time: " << poseGtBuf.front()->header.stamp.toNSec() << "\n";                                
                poseGtOneScanBuf.push(poseGtBuf.front());
                poseGtBuf.pop();
                 waitForEmptyQueue(poseGtBuf);
            }     

            // Exception for last frame
            if(poseGtOneScanBuf.size() < 17){
                break;
            }

            laserCloudIR = removeOverOneFrame(laserCloudIR);

            std::cout << "After OneFrame Cloud size: " << laserCloudIR->points.size() << "\n"; 

            // For debug
            // if(frameCount == 0){
            //     writeTxtForCloudCompare(laserCloudIR, "one_scan_before_removal.txt");
            //     laserCloudIR = removeOverOneFrame(laserCloudIR);
            //     writeTxtForCloudCompare(laserCloudIR, "one_scan_after_removal.txt");
            // }

            if(TO_BAG){
                pcl::PointCloud<pcl::PointXYZI>::Ptr refinedRawCloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointXYZI point;

                for(size_t i = 0; i < laserCloudIR->points.size(); i++){
                    point.x = laserCloudIR->points[i].x;
                    point.y = laserCloudIR->points[i].y;
                    point.z = laserCloudIR->points[i].z;
                    point.intensity = laserCloudIR->points[i].intensity;

                    refinedRawCloud->points.push_back(point);
                }

                sensor_msgs::PointCloud2 refinedRawCloud2;
                pcl::toROSMsg(*refinedRawCloud, refinedRawCloud2);
                refinedRawCloud2.header.stamp = currLaserStamp;
                refinedRawCloud2.header.frame_id = "/velodyne";
            
                bag_out.write("/velodyne_points", refinedRawCloud2.header.stamp, refinedRawCloud2);
            }
            

            size_t pointCount = 0;
            int poseCount = 0;
            Eigen::Quaterniond q_first;
            Eigen::Vector3d t_first;   
            // 1개 스캔의 포즈들에 따라 점군 매핑
            while(!poseGtOneScanBuf.empty()){

                if(oneFrameMappingDone){
                    break;
                }

                Eigen::Quaterniond q_curr;
                Eigen::Vector3d t_curr;
                poseTime = poseGtOneScanBuf.front()->header.stamp.toNSec();
                
                if(TO_BAG){
                    bag_out.write("/pose_ground_truth", poseGtOneScanBuf.front()->header.stamp, poseGtOneScanBuf.front());
                }

                if(poseCount == 0){
                    q_first.x() = poseGtOneScanBuf.front()->pose.orientation.x;
                    q_first.y() = poseGtOneScanBuf.front()->pose.orientation.y;
                    q_first.z() = poseGtOneScanBuf.front()->pose.orientation.z;
                    q_first.w() = poseGtOneScanBuf.front()->pose.orientation.w;
                    t_first.x() = poseGtOneScanBuf.front()->pose.position.x;
                    t_first.y() = poseGtOneScanBuf.front()->pose.position.y;
                    t_first.z() = poseGtOneScanBuf.front()->pose.position.z;
                    pcl::PointXYZ viewPoint;
                    viewPoint.x = t_first.x();
                    viewPoint.y = t_first.y();
                    viewPoint.z = t_first.z();
                    lidarViewPoint->points.push_back(viewPoint);
                }                

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
                
                size_t pointNumEnd;
                // scanEndInd에서 매핑해야하는 인덱스를 다썼거나, pose가 비어있는경우 매핑을 멈춤
                if(scanEndInd.size()-1 == poseCount || poseGtOneScanBuf.empty()){
                    pointNumEnd = laserCloudIR->points.size();
                    oneFrameMappingDone = true;
                }
                else{
                    pointNumEnd = scanEndInd[poseCount];
                }                
                
                for(size_t i = pointCount; i < pointNumEnd; i++){          
                    systemCalibration(&laserCloudIR->points[i], &laserCloudIR->points[i]);

                    if(MOTION_CORRECTED){
                        pointTransform(&laserCloudIR->points[i], &laserCloudIR->points[i], q_curr, t_curr);  
                    }
                    else{
                        pointTransform(&laserCloudIR->points[i], &laserCloudIR->points[i], q_first, t_first);
                    }  
                    
                    if(!printedFlag){
                        std::cout << "After Mapping Cloud size: " << laserCloudIR->points.size() << "\n";   
                        printedFlag = true;
                    }
                    lidarMapIR->points.push_back(laserCloudIR->points[i]);           
                    
                }

                poseCount++;
                pointCount = pointNumEnd;
            }

            printedFlag = false;
            oneFrameMappingDone = false;

            frameCount++;
            scanEndInd.clear();
        }

        while((laserCloudBuf.empty() || poseGtBuf.empty()) && startedFlag){
            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
            waitingMsec += 100;
            
            if(waitingMsec > 1000){
                
                bag_out.close();
                
                std::cout << "End of stream\nWriting ply file... \n";

                pcl::PLYWriter writer;
                writer.write<PointXYZIRD> 
                    (OUTPUT_PLY_FILE + "IRD.ply",*lidarMapIR, false);
                std::cout << "Ply writing done! \n Writing rgb ply file... \n";
               
                lidarMapIRVoxelFiltered = voxelFiltering(lidarMapIR);
                computeNormal(lidarMapIRVoxelFiltered);
                writeRgbMapForMeshLab(lidarMapIRVoxelFiltered);

                std::cout << "All done! Sir! \n";

                // Effect of Motion Distortion
                // writeTxtForCloudCompare(lidarMapIR, "lidarMap.txt");
                
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
    nh.getParam("to_bag", TO_BAG);
    nh.getParam("output_bag_file", OUTPUT_BAG_FILE);
    nh.getParam("to_meshlab_ply", TO_MESHLAB_PLY);
    nh.getParam("output_ply_file", OUTPUT_PLY_FILE);
    nh.getParam("distance_threshold", DIST_THRES);
    nh.getParam("voxel_leaf_size", LEAF_SIZE);
    nh.getParam("motion_corrected", MOTION_CORRECTED);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    auto str = oss.str();

    std::cout << "Current Date Time: "<< str << "\n";    
    
    if(TO_BAG){
        std::string bagFileName = OUTPUT_BAG_FILE + str + ".bag";
        bag_out.open(bagFileName, rosbag::bagmode::Write);
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                ("/velodyne_points", 100, laserCloudHandler);

    ros::Subscriber subGtPose = nh.subscribe<geometry_msgs::PoseStamped>
                                ("/pose_ground_truth", 100, poseGtHandler);
    
    std::thread mapping_process{process};

    ros::spin();

    return 0;
}