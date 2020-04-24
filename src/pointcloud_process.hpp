#include "ros/ros.h"
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include "pointcloud_process/boatpose.h"
#include "pointcloud_process/stereo_vision_msg.h"
#include "pointcloud_process/obstacal_msg.h"
#include "pointcloud_process/xyz_msg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <time.h>
#include "gnss/GPHCD_msg.h"
#include <thread>
#include <QVTKWidget.h>
#include <QtWidgets>
#include <pcl/common/centroid.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sys/time.h>
#include <fstream>
using namespace cv;  

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct usv_state
{
    float boat_heading = 0;    //船头方向
    float boat_roll = 0;       //船体摇摆
    float boat_pitch = 0;      //船体俯仰
    float boat_velocity = 0;   //船速
    float boat_velocity_x = 0; //船速东
    float boat_velocity_y = 0; //船速北
    float boat_velocity_z = 0; //船速上
    float boat_latitude = 0;   //纬度
    float boat_longitude = 0;
};

//定义障碍物类
class cloud_object
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr xcloud;//点云
    pcl::PointXYZ minPt, maxPt;//bounds
    PointT pccenter;//点云中心
    float cloud_radius=0.0;//点云半径
    PointT direction;//运动方向
    PointT direction_average;//运动平均方向
    int update_times=0.0;//连续丢失次数
    float update_frequency=0.0;//连续跟踪率
    float max_update_frequency=0.0;//连续跟踪率
    double timestam=0.0;
    float longitude=0.0;
    float latitude=0.0;
    bool High_probability_target = false;

    // std::vector<VectorXd> estimations;
    // std::vector<VectorXd> ground_truths;
    //计算平均速度
    double get_direction_average_length(void)
    {
        return sqrt(pow(direction_average.x, 2) + pow(direction_average.y, 2) + pow(direction_average.z, 2));
    }
    //计算目标高度
    float get_target_height(void)
    {
        return abs(maxPt.z - minPt.z);
    }
    //计算点云中心
    PointT getpointcloudcenter(void)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*xcloud, centroid);
        pccenter.x = centroid[0];
        pccenter.y = centroid[1];
        pccenter.z = centroid[2];
        return pccenter;
    }
    void set_update_frequency(float v)
    {
        update_frequency = v;
        if(update_frequency>max_update_frequency)
        {
            max_update_frequency=update_frequency;
            if(max_update_frequency>0.90) High_probability_target=true;
        }
    }

    void miss_update_position(float boat_velocity_x, float boat_velocity_y, float boat_velocity_z){
        pccenter.x = pccenter.x + direction_average.x - boat_velocity_x;
        pccenter.y = pccenter.y + direction_average.y - boat_velocity_y;
        pccenter.z = pccenter.z + direction_average.z  - boat_velocity_z;
    }

};