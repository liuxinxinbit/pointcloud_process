
#include "pointcloud_process.hpp"
#define _USE_MATH_DEFINES
#include <cmath>


using namespace std;
using namespace cv;

//参数初始化
std::string save_data_dir = "/home/liuxinxin/ToolKit/result"; //数据保存路径
#define Pi 3.14159265                                         //定义pi值
int pool_frame_num = 10;                                      //中值滤波帧数                                    //经度
long file_index = 0;                                          //激光雷达帧数
int cloud_size_limit = 0;                                     //目标点数限制
float ignore_distance = 20.0;                                 //忽视距离
float ClusterTolerance = 10.0;                                // 距离聚类阈值
int update_frequency_num = 15;                                //跟踪成功频率统计帧数
float update_frequency_threshold = 0.5;                       //跟踪成功频率阈值
int update_times_threshold = 3;                               //跟踪成功频率
float crop_radius = 20;                                       //截取半径
int global_detection_frequency = 10;                          //全局搜索频率
usv_state ship;
pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Received(new pcl::PointCloud<pcl::PointXYZ>); //激光雷达数据
pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Mask(new pcl::PointCloud<pcl::PointXYZ>);     //激光雷达数据

Mat mask_img(600, 600, CV_8UC1, Scalar(0));//mask图像定义

//障碍物追踪列表
vector<cloud_object> targetcloud_waitfortrack; //***
ros::ServiceClient encSrv;                     //enc服务客户端

int64_t getCurrentTime() //直接调用这个函数就行了，返回值最好是int64_t，long long应该也可以
{
    struct timeval tv;
    gettimeofday(&tv, NULL); //该函数在sys/time.h头文件中
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

//ICP 配准
float PCL_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    // input: two cloud data, output: icp registration result
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaximumIterations (100);
    // icp.setInputSource(cloud_in);
    // icp.setInputTarget(cloud_out);
    // pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final);
    // Eigen::Matrix4f Transformation_Matrix=icp.getFinalTransformation();
    // translation_matrix=icp.getFitnessScore();
    //*********************************************
    float fitcore = 1 - abs(cloud_in->points.size() - cloud_out->points.size()) / (cloud_in->points.size() + cloud_out->points.size());
    //********************************************
    return fitcore;
}
//极坐标转直角坐标
vector<float> Spherical2RectangularCoordinateSystem(float radius, float theta, float phi)
{
    //input: polar coordinates，output: Rectangular coordinates
    vector<float> scs;
    float x = radius * sin(theta) * cos(phi);
    float y = radius * sin(theta) * sin(phi);
    float z = radius * cos(theta);
    scs.push_back(x);
    scs.push_back(y);
    scs.push_back(z);
    return scs;
}
//直角坐标转极坐标
vector<float> Rectangular2SphericalCoordinateSystem(float x, float y, float z)
{
    //input: Rectangular coordinates, output: polar coordinates
    vector<float> scs;
    float r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    float theta = acos(z / r);
    float phi = atan2(y, x);
    scs.push_back(r);
    scs.push_back(theta);
    scs.push_back(phi);
    return scs;
}
//点云mask运算，去除非水面目标
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // enc::enc_srv enc; //enc服务消息，用于保存enc请求与响应数据
    // enc.request.position.longitude = boat_longitude;
    // enc.request.position.latitude  = boat_latitude;
    // enc.request.direction          = 0;
    // enc.request.range              = 1.278256;
    // static int cnt = -1;
    // if (cnt < 1) {
    //     if (!encSrv.call(enc)) {
    //         // ROS_INFO("CAll ENC SERVER FAILURE");
    //         if (cnt < 0) return cloud;
    //     } else {
    //         cnt = 0;
    //         int height = enc.response.enc.height;
    //         int width  = enc.response.enc.width;
    //         boost::scoped_ptr<IplImage> pSrc(cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1));
    //         for(int i = 0; i < height; ++i) {
    //             uchar* ptr = (uchar*) (pSrc->imageData + i * (pSrc->widthStep));
    //             for(int j = 0; j < width; ++j) {
    //                 ptr[j] = enc.response.enc.data[i * width + j];
    //             }
    //         }
    //         mask_img = cvarrToMat(pSrc.get());
    //         // Mat element = getStructuringElement(MORPH_ELLIPSE, Size(80, 80));
    //         // dilate(mask_img, mask_img, element);
    //     }
    // }
    // ++cnt;
    // cnt %= 10;

    //     // imshow("enc", mask_img);

    //电子还图mask操作
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZ p = cloud->points[i];
        vector<float> src = Rectangular2SphericalCoordinateSystem(p.x, p.y, p.z);
        if (mask_img.at<uint8_t>(Point(300 + p.x / 2.13, 300 - p.y / 2.13)) < 23 && p.z > -200.5)
        {
            float ang = ship.boat_heading + 180;
            if (ang >= 360)
                ang = ang - 360;
            if (abs(90 - (src[2] * 180 / Pi) - ang) > 90)
            {
                if (src[0] >= 20)
                    cloud_cluster->points.push_back(cloud->points[i]);
            }
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
    }
    Cloud_Mask = cloud_cluster;
    return cloud_cluster;
}

//向量长度
float Vector_length(float x, float y, float z)
{
    //input: vector, output: Vector length
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
//两点之间距离1
float point_length(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    //input: two point, output: the length between two point
    float x = p1.x - p2.x;
    float y = p1.y - p2.y;
    float z = p1.z - p2.z;
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
//两点之间距离1
float point_length(PointT p)
{
    //input: two point, output: the length between two point
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}

//目标类初始化
cloud_object initial_cloud_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //input: cloud data, output: Obstacle object
    cloud_object co;
    co.xcloud = cloud;
    pcl::getMinMax3D(*cloud, co.minPt, co.maxPt);
    co.getpointcloudcenter();
    co.cloud_radius = max(Vector_length(co.minPt.x - co.pccenter.x, co.minPt.y - co.pccenter.y, co.minPt.z - co.pccenter.z),
                          Vector_length(co.maxPt.x - co.pccenter.x, co.maxPt.y - co.pccenter.y, co.maxPt.z - co.pccenter.z));
    PointT p=co.getpointcloudcenter();
    co.direction.x = 0;
    co.direction.y = 0;
    co.direction.z = 0;
    co.direction_average.x = 0;
    co.direction_average.y = 0;
    co.direction_average.z = 0;
    co.update_times = 0;
    co.set_update_frequency(0);
    co.max_update_frequency = 0;
    co.timestam = getCurrentTime();
    co.longitude = 0.0;
    co.latitude = 0.0;
    return co;
}

//距离聚类的点云分割
vector<cloud_object> pointCloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //input: cloud data, output: Obstacle list
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(ClusterTolerance); // 10m
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    vector<cloud_object> screen_targt_cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        std::vector<int> indices = it->indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_object co = initial_cloud_object(cloud_cluster);
        if (co.get_target_height() > 1.5 || point_length(co.pccenter) > 50)
            screen_targt_cloud.push_back(co);
    }
    return screen_targt_cloud;
}

//统计噪声点去除
pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //input: origin cloud data, output: refine cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(5);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

//点云平移旋转转换
pcl::PointCloud<PointT>::Ptr pointcloud_tansform(pcl::PointCloud<PointT>::Ptr cloud, float roll, float pitch, float yaw)
{
    //Point cloud rotation theta_
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Quaternion<float> quaternion;
    Eigen::Quaternion<float> qx = Eigen::Quaternion<float>(cos(pitch / 2.0), sin(pitch / 2.0), 0, 0);
    Eigen::Quaternion<float> qy = Eigen::Quaternion<float>(cos(roll / 2.0), 0, sin(roll / 2.0), 0);
    Eigen::Quaternion<float> qz = Eigen::Quaternion<float>(cos(yaw / 2.0), 0, 0, -sin(yaw / 2.0));
    quaternion = (qy * qx * qz);
    quaternion.normalize();
    auto trans = quaternion.toRotationMatrix().transpose();
    transform(0, 0) = trans(0, 0);
    transform(0, 1) = trans(0, 1);
    transform(0, 2) = trans(0, 2);
    transform(1, 0) = trans(1, 0);
    transform(1, 1) = trans(1, 1);
    transform(1, 2) = trans(1, 2);
    transform(2, 0) = trans(2, 0);
    transform(2, 1) = trans(2, 1);
    transform(2, 2) = trans(2, 2);
    pcl::transformPointCloud(*cloud, *cloud, transform);
    return cloud;
}
//点云指定半径邻域截取
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_crop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ searchPoint, float radius)
{
    //input: cloud data and crop radius, output: part cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建kdtree
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        crop_cloud->width = pointIdxRadiusSearch.size();
        crop_cloud->height = 1;
        crop_cloud->points.resize(crop_cloud->width * crop_cloud->height);
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            crop_cloud->points[i] = cloud->points[pointIdxRadiusSearch[i]];
        }
    }
    return crop_cloud;
}
//判断向量中是否有目标
bool _find(int value, vector<int> vec)
{
    bool flag = false;
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (value == i)
            flag = true;
    }
    return flag;
}
//Obstacle List Update
vector<cloud_object> targetcloud_waitfortrack_update(
    vector<cloud_object> targetcloud_waitfortrack, vector<cloud_object> targetcloud_waitfortrack_new)
{
    //input:origin obstacle, output: updated obstacle
    vector<int> target_index_vector;
    int target_index;
    for (size_t i = 0; i < targetcloud_waitfortrack.size(); i++)
    {//搜索最佳匹配搜索
        cloud_object targetcloud_best;
        float fitscore = -99999.0;
        bool flag = false;
        for (size_t j = 0; j < targetcloud_waitfortrack_new.size(); j++)
        {
            float length = Vector_length(
                targetcloud_waitfortrack[i].pccenter.x - targetcloud_waitfortrack_new[j].pccenter.x,
                targetcloud_waitfortrack[i].pccenter.y - targetcloud_waitfortrack_new[j].pccenter.y,
                targetcloud_waitfortrack[i].pccenter.z - targetcloud_waitfortrack_new[j].pccenter.z);
            if (length > crop_radius)
                continue;
            float transformvector = PCL_ICP(targetcloud_waitfortrack_new[j].xcloud, targetcloud_waitfortrack[i].xcloud);
            if (transformvector > fitscore)
            {
                fitscore = transformvector;
                targetcloud_best = targetcloud_waitfortrack_new[j];
                flag = true;
                target_index = j;
            }
        }
        //更新目标信息
        if (flag)//发现目标
        {
            if (target_index >= 0) target_index_vector.push_back(target_index);
            targetcloud_best.update_times = 0;
            targetcloud_best.set_update_frequency((targetcloud_waitfortrack[i].update_frequency * update_frequency_num + 1) / (update_frequency_num + 1));

            targetcloud_best.direction.x = targetcloud_best.pccenter.x - targetcloud_waitfortrack[i].pccenter.x + ship.boat_velocity_x;
            targetcloud_best.direction.y = targetcloud_best.pccenter.y - targetcloud_waitfortrack[i].pccenter.y + ship.boat_velocity_y;
            targetcloud_best.direction.z = targetcloud_best.pccenter.z - targetcloud_waitfortrack[i].pccenter.z + ship.boat_velocity_z;

            targetcloud_best.direction_average.x =
                (targetcloud_waitfortrack[i].direction_average.x * pool_frame_num + targetcloud_best.direction.x) / (pool_frame_num + 1);
            targetcloud_best.direction_average.y =
                (targetcloud_waitfortrack[i].direction_average.y * pool_frame_num + targetcloud_best.direction.y) / (pool_frame_num + 1);
            targetcloud_best.direction_average.z =
                (targetcloud_waitfortrack[i].direction_average.z * pool_frame_num + targetcloud_best.direction.z) / (pool_frame_num + 1);
            targetcloud_waitfortrack[i] = targetcloud_best;
        }
        else//为发现目标
        {
            targetcloud_best = targetcloud_waitfortrack[i];
            targetcloud_best.update_times = targetcloud_waitfortrack[i].update_times + 1;
            targetcloud_best.set_update_frequency((targetcloud_waitfortrack[i].update_frequency * update_frequency_num + 0) / (update_frequency_num + 1));
            targetcloud_best.miss_update_position(ship.boat_velocity_x,ship.boat_velocity_y,ship.boat_velocity_z);
            targetcloud_waitfortrack[i] = targetcloud_best;
        }
    }
    for (size_t i = 0; i < targetcloud_waitfortrack_new.size(); i++)
    {
        if (!_find(i, target_index_vector))
        {
            targetcloud_waitfortrack.push_back(targetcloud_waitfortrack_new[i]);
        }
    }
    vector<cloud_object> targetcloud_waitfortrack_fanal;
    for (size_t i = 0; i < targetcloud_waitfortrack.size(); i++)
    {
        // if (targetcloud_waitfortrack[i].update_times < 5)
            targetcloud_waitfortrack_fanal.push_back(targetcloud_waitfortrack[i]);
    }
    return targetcloud_waitfortrack_fanal;
}

//全局搜索结果处理
void cloud_target_screen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //input: cloud data, output: serch obstacle from entire point cloud
    if (targetcloud_waitfortrack.size() == 0)//目标列表有目标
    {
        targetcloud_waitfortrack = pointCloud_cluster(cloud);
    }
    else//目标列表无目标
    {
        vector<cloud_object> targetcloud_waitfortrack_new = pointCloud_cluster(cloud);
        targetcloud_waitfortrack = targetcloud_waitfortrack_update(targetcloud_waitfortrack, targetcloud_waitfortrack_new);
    }
}

//局部目标跟踪
cloud_object target_pointcloud_direction_singlecloud(cloud_object cloud_model, vector<cloud_object> pointcloud_vector)
{
    float maxcore = -99999.9;//最大相似度
    cloud_object cloud_object_best;//定义最佳目标
    bool flag = false;
    if (pointcloud_vector.size() > 0)//挑选最相似度问题
    {
        for (size_t target_index = 0; target_index < pointcloud_vector.size(); target_index++)
        {
            float fitcore = PCL_ICP(cloud_model.xcloud, pointcloud_vector[target_index].xcloud);//匹配分数
            if (fitcore > maxcore)
            {
                maxcore = fitcore;
                cloud_object_best = pointcloud_vector[target_index];
                flag = true;
            }
        }
    }
    // if (true)//发现目标
    // {
        cloud_object_best.update_times = 0;
        cloud_object_best.set_update_frequency((cloud_model.update_frequency * update_frequency_num + 1) / (update_frequency_num + 1));

        cloud_object_best.direction.x = cloud_object_best.pccenter.x - cloud_model.pccenter.x + ship.boat_velocity_x;
        cloud_object_best.direction.y = cloud_object_best.pccenter.y - cloud_model.pccenter.y + ship.boat_velocity_y;
        cloud_object_best.direction.z = cloud_object_best.pccenter.z - cloud_model.pccenter.z + ship.boat_velocity_z;

        cloud_object_best.direction_average.x =
            (cloud_model.direction_average.x * pool_frame_num + cloud_object_best.direction.x) / (pool_frame_num + 1);
        cloud_object_best.direction_average.y =
            (cloud_model.direction_average.y * pool_frame_num + cloud_object_best.direction.y) / (pool_frame_num + 1);
        cloud_object_best.direction_average.z =
            (cloud_model.direction_average.z * pool_frame_num + cloud_object_best.direction.z) / (pool_frame_num + 1);
    return cloud_object_best;
}

//去除目标列表中的重复目标
void remove_repeat_target()
{
    vector<cloud_object> refine_target_vector;
    refine_target_vector.push_back(targetcloud_waitfortrack[0]);
    for (size_t i = 1; i < targetcloud_waitfortrack.size(); i++)
    {
        bool flag = true;
        for (size_t j = 0; j < refine_target_vector.size(); j++)
        {
            if (point_length(targetcloud_waitfortrack[i].pccenter, refine_target_vector[j].pccenter) < 10)
                flag = false;
        }
        if (flag)
            refine_target_vector.push_back(targetcloud_waitfortrack[i]);
    }
    targetcloud_waitfortrack = refine_target_vector;
}
void remove_longtime_target(){
    vector<cloud_object> refine_target_vector;
    double secs =getCurrentTime();
    // ROS_INFO("time: %f",    secs);
    for (size_t i = 0; i < targetcloud_waitfortrack.size(); i++)
    {
        // ROS_INFO("time22: %f",    secs-targetcloud_waitfortrack[i].timestam);
        if (secs-targetcloud_waitfortrack[i].timestam<10000)
            refine_target_vector.push_back(targetcloud_waitfortrack[i]);
    }
    targetcloud_waitfortrack = refine_target_vector;
}

//点云数据加入处理流程
void add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //save data as pcd
    // std::string file_head="/lidar_data_";
    // std::string file_tail=".pcd";
    // std::string file_name=save_data_dir+file_head+std::to_string(file_index)+file_tail;
    // //ROS_INFO("save file %s", file_name.c_str());
    // if(cloud->points.size()>0)
    // pcl::io::savePCDFileASCII (file_name, *cloud);
    //**************************

    if (file_index % global_detection_frequency == 0)
    {
        cloud_target_screen(cloud);//全局搜索
    }
    else
    {
        for (size_t cloud_index = 0; cloud_index < targetcloud_waitfortrack.size(); cloud_index++)
        {
            //局部搜索
            cloud_object target = targetcloud_waitfortrack[cloud_index];
            pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (cloud->points.size() > cloud_size_limit)
            {
                crop_cloud = cloud_radius_crop(cloud, target.pccenter, crop_radius);//局部截取
                
                if (crop_cloud->points.size() >0)
                {
                    vector<cloud_object> pointcloud_screen = pointCloud_cluster(crop_cloud);//聚类分割
                    cloud_object target_update = target_pointcloud_direction_singlecloud(target, pointcloud_screen);
                    // if (target_update.update_times < update_times_threshold)
                    targetcloud_waitfortrack[cloud_index] = target_update;
                }
                else{
                    targetcloud_waitfortrack[cloud_index] = target;
                    targetcloud_waitfortrack[cloud_index].update_times = target.update_times + 1;
                     targetcloud_waitfortrack[cloud_index].set_update_frequency((target.update_frequency * update_frequency_num + 0) / (update_frequency_num + 1));
                     targetcloud_waitfortrack[cloud_index].miss_update_position(ship.boat_velocity_x,ship.boat_velocity_y,ship.boat_velocity_z);

                }
            }
        }
    }
    if (targetcloud_waitfortrack.size() > 0)
        remove_repeat_target();
        remove_longtime_target();
}

//目标抚养校正
vector<float> CoordinateSystem_transform(float x, float y, float z)
{
    vector<float> rsc = Rectangular2SphericalCoordinateSystem(x, y, z);
    vector<float> src = Spherical2RectangularCoordinateSystem(rsc[0], Pi / 2, rsc[2]);
    return src;
}
//去除跟踪次数小于指定次数的目标
vector<cloud_object> target_refine(vector<cloud_object> t_w)
{
    vector<cloud_object> n_t_w;
    for (size_t i = 0; i < t_w.size(); i++)
    {
        if(t_w[i].High_probability_target)
        {
            if (t_w[i].update_frequency > 0.0 && point_length(t_w[i].pccenter) > ignore_distance)
            n_t_w.push_back(t_w[i]);
        }
        else{
            if (t_w[i].update_frequency > update_frequency_threshold && point_length(t_w[i].pccenter) > ignore_distance)
            n_t_w.push_back(t_w[i]);
        }
        
    }
    return n_t_w;
}



//生成目标障碍物信息
pointcloud_process::stereo_vision_msg generate_msg()
{
    vector<cloud_object> targetcloud_waitfortrack_msg = target_refine(targetcloud_waitfortrack);
    pointcloud_process::stereo_vision_msg msg;
    msg.obstacals.resize(targetcloud_waitfortrack_msg.size());
    float volocity, distance;
    for (size_t i = 0; i < targetcloud_waitfortrack_msg.size(); i++)
    {
        cloud_object co = targetcloud_waitfortrack_msg[i];
        vector<float> scs_center = CoordinateSystem_transform(co.pccenter.x, co.pccenter.y, co.pccenter.z);
        
        vector<float> rsc = Rectangular2SphericalCoordinateSystem(co.direction_average.x, co.direction_average.y, co.direction_average.z);
        volocity = co.get_direction_average_length();//计算速度
        distance = sqrt(pow(scs_center[0], 2) + pow(scs_center[1], 2) + pow(scs_center[2], 2));//计算距离
        double earth_radius = 2*Pi*6371393;//地球周长
        msg.obstacals[i].radius = co.cloud_radius;//目标半径
        msg.obstacals[i].position.x = scs_center[0];//cos(boat_latitude)*scs_center[0]/earth_radius+boat_longitude;//目标相对位置东
        msg.obstacals[i].position.y = scs_center[1];///earth_radius+boat_latitude;//目标相对位置北
        msg.obstacals[i].position.z = scs_center[2];//目标相对位置上
        msg.obstacals[i].velocity.x = co.direction_average.x * 10;//速度东
        msg.obstacals[i].velocity.y = co.direction_average.y * 10;//速度北
        msg.obstacals[i].velocity.z = co.direction_average.z;//速度上
        msg.obstacals[i].probability=co.update_frequency;
        // ROS_INFO("Tracking:  E:%12.7f N:%12.7fVE:%12.7f VN:%12.7f ",    msg.obstacals[i].position.x, msg.obstacals[i].position.y, msg.obstacals[i].velocity.x, msg.obstacals[i].velocity.y );
        // ROS_INFO("Boat:  lon:%12.7f lat:%12.7fheading:%12.7f velocity:%12.7f ",    boat_longitude, boat_latitude, boat_heading,boat_velocity);
    }
    return msg;
}
//opencv mat 画箭头
void drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar &color, int thickness, int lineType)
{
    const double PI = 3.1415926;
    Point arrow;
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));

    line(img, pStart, pEnd, color, thickness, lineType);

    //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置）
    arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);

    arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);

    line(img, pEnd, arrow, color, thickness, lineType);

    arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);

    arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);

    line(img, pEnd, arrow, color, thickness, lineType);
}

void drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
               cv::Scalar &color, int thickness = 1, int lintType = 8);

//显示结果
void show_result(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Received)
{
    int img_size=400;
    vector<cloud_object> targetcloud_waitfortrack_msg = target_refine(targetcloud_waitfortrack); //优化结果
    Mat img(img_size, img_size, CV_8UC3, Scalar(0, 0, 0));                                                 //定义结果图片
    // mask_img.copyTo(img);
    // Mat img = mask_img;
    line(img, Point(img_size/2-50, img_size/2), Point(img_size/2+50, img_size/2), Scalar(0, 255, 0)); //x轴线
    // line(img, Point(img_size/2, img_size/2-50), Point(img_size/2, img_size/2+50), Scalar(0, 255, 0)); //y轴线
    circle(img, Point((img_size/2 ), (img_size/2)), 200, cv::Scalar(0, 255, 255));
    //点云转换成二维图像
    for (size_t i = 0; i < Cloud_Received->points.size(); i++)
    {
        int x = (int)Cloud_Received->points[i].x;// / 2.13;
        int y = (int)Cloud_Received->points[i].y ;/// 2.13;
        int color = (Cloud_Received->points[i].z + 3) * 80;
        color = color > 255 ? 255 : color;
        circle(img, Point((img_size/2 + x), (img_size/2 - y)), 1, cv::Scalar(255, 255, 255));
    }

            int font_face = cv::FONT_HERSHEY_COMPLEX;
            double font_scale = 0.5;
            int thickness = 0.3;
    Point pStart((img_size/2), (img_size/2)), pEnd((img_size/2 + 0), (img_size/2 - 25));//箭头起始点
    Scalar lineColor(255, 255, 255);
       drawArrow(img, pStart, pEnd, 10, 45,  lineColor);//画箭头
    for (size_t index = 0; index < targetcloud_waitfortrack_msg.size(); index++)
    {
        vector<float> scs_center = CoordinateSystem_transform(targetcloud_waitfortrack_msg[index].pccenter.x,
                                                              targetcloud_waitfortrack_msg[index].pccenter.y,
                                                              targetcloud_waitfortrack_msg[index].pccenter.z);//目标中心
        vector<float> rsc = Rectangular2SphericalCoordinateSystem(targetcloud_waitfortrack_msg[index].pccenter.x,
                                                                  targetcloud_waitfortrack_msg[index].pccenter.y,
                                                                  targetcloud_waitfortrack_msg[index].pccenter.z);//坐标转换

        double x = targetcloud_waitfortrack_msg[index].pccenter.x;// / 2.13;//中心点位置x
        double y = targetcloud_waitfortrack_msg[index].pccenter.y;// / 2.13;//中心点位置y
        circle(img, Point((img_size/2 + x), (img_size/2 - y)), 6, cv::Scalar(0, 0, 255));//画点
        double yy = targetcloud_waitfortrack_msg[index].direction_average.y;// / 2.13;//位移北
        double xx = targetcloud_waitfortrack_msg[index].direction_average.x ;/// 2.13;//位移东
        Point pStart((img_size/2 + x), (img_size/2 - y)), pEnd((img_size/2 + x + xx * 50), (img_size/2 - y - yy * 50));//箭头起始点
        Scalar lineColor(0, 0, 255);
        drawArrow(img, pStart, pEnd, 10, 45, lineColor);//画箭头
        int update_frequency_text = targetcloud_waitfortrack_msg[index].update_frequency*100;
        std::string text = std::to_string(update_frequency_text);
        cv::putText(img, text, pStart, font_face, font_scale, cv::Scalar(0,255,255));
    }
    // 显示结果并保存图片
    cv::namedWindow("Lidar window", CV_WINDOW_NORMAL);
    cv::resizeWindow("Lidar window", 800, 800);
    cv::imshow("Lidar window", img);
    

    // std::string file_name = save_data_dir + file_head + std::to_string(file_index)
    // std::string file_head = "/lidar_data_";
    // std::string file_tail = ".png";
    // std::string file_name = save_data_dir + file_head + std::to_string(file_index) + file_tail;
    // if(file_index%20==0)
    // cv::imwrite(file_name, img);

    waitKey(1);
}
//Test Timing Function
void test()
{

    for (int i = 0; i < 1897; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        stringstream ss;
        ss << "/home/liuxinxin/data/1/lidar_data_" << i << ".pcd";
        cout << ss.str() << endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(), *cloud) == -1)
        {
            PCL_ERROR("Couldn't read target pcd file \n");
        }
        Cloud_Mask = cloud_roi(cloud);
        add_cloud(Cloud_Mask);
        file_index++;
        show_result(Cloud_Mask);
    }
}
//接收和发布ros消息类
class SubscribeAndPublish
{
public:
    //订阅激光雷达和定位信息，定义目标信息发布器
    SubscribeAndPublish()
    {
        // encSrv = nh.serviceClient<enc::enc_srv>("Enc_nodess/Enc_srv");
        sub_pointcloud= nh.subscribe("/pandar_points", 1, &SubscribeAndPublish::getpointcloudcallback, this);
        sub_boatpose = nh.subscribe("GNSS_node/position_msg", 1, &SubscribeAndPublish::boatposecallback, this);
        sub_boatpose2 = nh.subscribe("IMU_node/position_msg", 1, &SubscribeAndPublish::boatposecallback2, this);
        msg_pub = nh.advertise<pointcloud_process::stereo_vision_msg>("pointcloud_tracking", 1);
    }
    //接收rtk信息，获得船艇gps消息，经度纬度俯仰摇摆等信息
    void boatposecallback(const gnss::GPHCD_msg::ConstPtr &msg)
    {
        ship.boat_heading = msg->heading;            //船向
        ship.boat_roll = msg->roll;                  // + 2.0f;//摇摆
        ship.boat_pitch = msg->pitch;                //俯仰
        ship.boat_velocity = msg->velocity;          //船速
        ship.boat_velocity_x = msg->velocity_e / 10; //船速东
        ship.boat_velocity_y = msg->velocity_n / 10; //船速北
        ship.boat_velocity_z = msg->velocity_u / 10; //船速上
        ship.boat_latitude = msg->latitude;          //纬度
        ship.boat_longitude = msg->longitude;        //经度
    }
    void boatposecallback2(const gnss::GPHCD_msg::ConstPtr& msg)
    {
        ship.boat_heading = msg->heading;
    }
    //节点接收激光雷达点云数据，加入点云数据处理流，产生结果发布消息
    void getpointcloudcallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // clock_t startTime, endTime;
        // startTime = clock();                  //计时开始
        pcl::PointCloud<pcl::PointXYZ> Cloud; //定义点云数据
        pcl::fromROSMsg(*msg, Cloud);         //激光雷达消息转换为点云
        //正北方向校正
        *Cloud_Received = Cloud;
        float toRad = Pi / 180;
        Cloud_Received = pointcloud_tansform(Cloud_Received, ship.boat_roll * toRad, ship.boat_pitch * toRad, Pi - ship.boat_heading * toRad);
        //基于海图mask操作
        Cloud_Received = cloud_roi(Cloud_Received);
        //开始目标检测和跟踪
        if (Cloud_Received->points.size() < 1000)
            add_cloud(Cloud_Received);
        //如果检测到目标，把检测跟踪结果发布
        if (targetcloud_waitfortrack.size() >= 0)
        {
            pointcloud_process::stereo_vision_msg obstacal_msg = generate_msg(); //生成消息
            msg_pub.publish(obstacal_msg);                                       //发布消息
        }
        show_result(Cloud_Received);
        file_index++;

    }

private:
    ros::NodeHandle nh;
    ros::Publisher msg_pub;         //目标检测和跟踪结果发布器
    ros::Subscriber sub_pointcloud; //激光雷达点云订阅器
    ros::Subscriber sub_boatpose;   //船体定位信息订阅器
    ros::Subscriber sub_boatpose2;   //船体定位信息订阅器
};

int main(int argc, char **argv)
{
    // test();
    //command_parameter_initial
    if(argc>9)
    {
        pool_frame_num = atoi(argv[1]);             //中值滤波帧数
        cloud_size_limit = atoi(argv[2]);           //目标点数限制
        ignore_distance = atoi(argv[3]);            //忽视距离
        ClusterTolerance = atoi(argv[4]);           // 距离聚类阈值
        update_frequency_num = atoi(argv[5]);       //跟踪成功频率统计帧数
        update_frequency_threshold = atof(argv[6]); //跟踪成功频率阈值
        update_times_threshold = atoi(argv[7]);     //跟踪成功频率
        crop_radius = atoi(argv[8]);                //截取半径
        global_detection_frequency = atoi(argv[9]); //全局搜索频率
    }
    ros::init(argc, argv, "Lidar_pointcloud_process");
    cout << "lidar tracking start!" << endl;
    //正式实施激光雷达目标检测跟踪
    SubscribeAndPublish sap;
    ros::spin();
    return 0;
}
