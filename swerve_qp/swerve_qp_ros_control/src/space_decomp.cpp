#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "space_decomp.hpp"

#include <octomap/octomap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <jps_planner/jps_planner/jps_planner.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <octomap_msgs/conversions.h>
#include <crop_sphere.hpp>
#include <geometry_msgs/PointStamped.h>
decomp_util::decomp_util(ros::NodeHandle nh): nh_(nh)
{
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
    path_pub = nh_.advertise<nav_msgs::Path>("path", 1, true);
    es_pub = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoids", 1, true);
    poly_pub = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
    GridMap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("GridMap", 1, true);
    Octomap_pub = nh_.advertise<octomap_msgs::Octomap>("octomap", 1, true);
    JPS_path_pub = nh_.advertise<nav_msgs::Path>("JPS_path", 1, true);
    JPS_3d_path_pub = nh_.advertise<nav_msgs::Path>("JPS_3D_path", 1, true);
    base_goal_pub = nh.advertise<geometry_msgs::PointStamped>("base_goal", 10);
    ee_goal_pub = nh.advertise<geometry_msgs::PointStamped>("ee_goal", 10);
    map_resolution_ = 0.1;
    map_util_ = std::make_shared<OccMapUtil>();
    map_util_3d_ = std::make_shared<VoxelMapUtil>();
    map_size_ = {15, 15, 10};
}

pcl::PointCloud<pcl::PointXYZ> decomp_util::readPCDFile(const std::string& file_name) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
    }
    return cloud;
}

void map_dilation (std::vector<int8_t>& gridData, double dilation_radius, int gridWidth, int gridHeight, double map_resolution)
{
    std::vector<std::vector<int8_t>> grid(gridHeight, std::vector<int8_t>(gridWidth));
    
    for (int i = 0; i < gridHeight; ++i) {
        for (int j = 0; j < gridWidth; ++j) {
            grid[i][j] = gridData[i * gridWidth + j];
        }
    }
    
    int dilation_num = static_cast<int>(std::ceil(dilation_radius / map_resolution));
    std::vector<std::vector<int8_t>> inflated_grid = grid;

    for (int i = 0; i < gridHeight; ++i) {
        for (int j = 0; j < gridWidth; ++j) {
            if (grid[i][j] == 100) {  // 100 indicates an obstacle
                for (int di = -dilation_num; di <= dilation_num; ++di) {
                    for (int dj = -dilation_num; dj <= dilation_num; ++dj) {
                        int ni = i + di;
                        int nj = j + dj;
                        if (ni >= 0 && ni < gridHeight && nj >= 0 && nj < gridWidth) {
                            // Check if the current cell is within the dilation circle
                            if (std::sqrt(di * di + dj * dj) <= dilation_num) {
                                inflated_grid[ni][nj] = 100;  // Expand the obstacle
                            }
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < gridHeight; ++i) {
        for (int j = 0; j < gridWidth; ++j) {
            gridData[i * gridWidth + j] = inflated_grid[i][j];
        }
    }
}



void decomp_util::decomp_and_publish(vec_Vec3f path, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    auto ee_pos = path.back();
    auto base_center = 0.5 * ( path[0] + path[1]);
    ros::WallTime start_time = ros::WallTime::now();
    octomap::OcTree tree(map_resolution_);

    for (const auto& point : cloud) {
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    tree.updateInnerOccupancy();

    // 设置2D栅格地图的参数
    double min_z = 0.0;
    double max_z = 0.3;

    // std::vector<double> minPoint(3, 0.0);
    // std::vector<double> maxPoint(3, 0.0);
    // tree.getMetricMin(minPoint[0], minPoint[1], minPoint[2]);
    // tree.getMetricMax(maxPoint[0], maxPoint[1], maxPoint[2]);
    // 创建 2D 栅格地图
    int gridWidth = map_size_[0]/ tree.getResolution();
    int gridHeight = map_size_[1]/ tree.getResolution();
    auto treedept = tree.getTreeDepth();
    
    sensor_msgs::PointCloud ROS_cloud;
    geometry_msgs::Point32 point;
    //std::vector<int8_t> gridData(gridWidth * gridHeight, -1);  // 初始化为未知区域
    gridData_ = std::vector<int8_t>(gridWidth * gridHeight, 0);  // 初始化为未知区域
    double map_x = (floor(base_center[0] / tree.getResolution()- gridWidth/2 )) * tree.getResolution();
    double map_y = (floor(base_center[1] / tree.getResolution()- gridWidth/2 )) * tree.getResolution();
    for (octomap::OcTree::iterator  it = tree.begin(), end = tree.end(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {

            octomap::point3d nodeCenter = it.getCoordinate();
            point.x = nodeCenter.x();
            point.y = nodeCenter.y();
            point.z = nodeCenter.z();
            ROS_cloud.points.push_back(point);
            auto depth = it.getDepth();
            double node_res = std::pow(2, treedept - depth) * tree.getResolution();
            auto node_zero = nodeCenter - octomap::point3d(node_res / 2, node_res / 2, node_res / 2);

            if (node_zero.z() >= min_z && node_zero.z() <= max_z) {
                int grid_x = static_cast<int>(std::ceil((node_zero.x() - map_x -tree.getResolution()/2) / tree.getResolution())) ;

                int grid_y = static_cast<int>(std::ceil((node_zero.y() - map_y -tree.getResolution()/2) / tree.getResolution()))  ;
                
                for (int i = 0; i <= treedept - depth; i++) {
                    for (int j = 0; j <= treedept - depth; j++) {
                        int index = (grid_y + j) * gridWidth + grid_x + i;
                        if ( index >= 0 && index < gridWidth * gridHeight) {
                            gridData_[index] = 100;  }
                }
                }
        }
    }}  
    // 发布 OctoMap
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::binaryMapToMsg(tree, octomap_msg);
    octomap_msg.header.frame_id = "odom"; // 根据需要设置
    octomap_msg.header.stamp = ros::Time::now();
    Octomap_pub.publish(octomap_msg);
    
   
    map_dilation(gridData_, 0.2, gridWidth, gridHeight, tree.getResolution());
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header.frame_id = "odom";
    occupancyGrid.header.stamp = ros::Time::now();
    occupancyGrid.info.resolution = tree.getResolution();
    occupancyGrid.info.width = gridWidth;
    occupancyGrid.info.height = gridHeight;
    occupancyGrid.info.origin.position.x = map_x;
    occupancyGrid.info.origin.position.y = map_y;
    occupancyGrid.info.origin.position.z = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.data = gridData_;
    GridMap_pub.publish(occupancyGrid);

    map_util_->setMap(Vec2f(map_x , map_y), Vec2i(gridWidth, gridHeight), gridData_, map_resolution_);
    map_util_3d_->readMap(cloud.makeShared(), map_size_[0]/map_resolution_, map_size_[1]/map_resolution_, map_size_[2]/map_resolution_, map_resolution_, Vec3f(ee_pos[0], ee_pos[1], ee_pos[2]), -3, 6, 0.0);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);


    
    sensor_msgs::convertPointCloud2ToPointCloud(output, ROS_cloud);
    ROS_cloud.header.frame_id = "odom";
    cloud_pub.publish(ROS_cloud);
    ros::WallTime map_time = ros::WallTime::now();

    vec_Vec3f obs = DecompROS::cloud_to_vec(ROS_cloud);

  // Read the point cloud from PCD file
 
    nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "odom";
    path_pub.publish(path_msg);
   
    // Using ellipsoid decomposition
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(obs);
    auto bboxs = std::vector<Vec3f>(path.size()-1, Vec3f(0.4, 0.3, 0.3));
    bboxs[0] = Vec3f(0.8, 0.8, 0.8);
    decomp_util.set_local_bbox(Vec3f(0.8, 0.8, 0.8));
    decomp_util.set_local_bboxs(bboxs);
    decomp_util.set_inflate_distance(0.02);
    decomp_util.dilate(path); // Set max iteration number of 10, do fix the path

    // Publish visualization msgs
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "odom";
    es_pub.publish(es_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "odom";
    poly_pub.publish(poly_msg);
    ros::WallTime end_time = ros::WallTime::now();
    std::cout << "Decomposition done!" << std::endl;
    std::cout << "Map time: " << (map_time - start_time).toSec() << " s" << std::endl;
    std::cout << "Decomp time: " << (end_time - map_time).toSec() << " s" << std::endl;
    LinearConstraints_.clear();
    freeregionCenters.clear();
    auto polys = decomp_util.get_polyhedrons();
    for(size_t i = 0; i < path.size() - 1; i++) {
        const auto pt_inside = (path[i] + path[i+1]) / 2;
        freeregionCenters.push_back(pt_inside);
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());

        // Now add the constraint "Above the ground:"
        // cs.A_.conservativeResize(cs.A_.rows() + 1, cs.A_.cols());
        // cs.A_.row(cs.A_.rows() - 1) = -Eigen::Vector3d::UnitZ();
        // cs.b_.conservativeResize(cs.b_.rows() + 1, cs.b_.cols());
        // cs.b_[cs.b_.rows() - 1] = -0.05;

        

        LinearConstraints_.push_back(cs);
    }
    // polys = decomp_util.get_polyhedrons();


}

void  decomp_util::decomp_and_publish(vec_Vec3f path, pcl::PointCloud<pcl::PointXYZ>& cloud, double radiu) {
    ros::WallTime start_time = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    CropSphere<pcl::PointXYZ> crop_sphere;
    crop_sphere.setInputCloud(cloud.makeShared());
    

    auto base_center =0.5*( path[0] + path[1]);
    Eigen::Vector4f center(base_center[0], base_center[1], base_center[2], 1.0);
    crop_sphere.setCenter(center);

    crop_sphere.setRadius(radiu);

    // 执行裁剪
    crop_sphere.filter(*cloud_filtered);
    
    ;
    ros::WallTime end_time = ros::WallTime::now();
    // std::cout << "Crop time: " << (end_time - start_time).toSec() << " s" << std::endl;
    decomp_and_publish(path, *cloud_filtered);
    map_size_ = {2*radiu, 2*radiu, 2*radiu};
}

// std::vector<LinearConstraint3D> decomp_util::getLinearConstraints() {
//   return LinearConstraints_;
// }

// std::vector<Vec3f> decomp_util::getFreeRegionCenters() {
//   return freeregionCenters;
// }
vec_Vec2f decomp_util::jps_planner(Vec2f start, Vec2f goal)
{
    std::unique_ptr<JPSPlanner2D> planner_ptr(new JPSPlanner2D(true)); // Declare a planner
    const Veci<2> start_int = map_util_->floatToInt(start);
    const Veci<2> goal_int = map_util_->floatToInt(goal);
    
      map_util_->setFreeVoxelAndSurroundings2D(start_int, 0.3);
      map_util_->setFreeVoxelAndSurroundings2D(goal_int, 0.3);

    planner_ptr->setMapUtil(map_util_); // Set collision checking function
    planner_ptr->updateMap();
    std::cout << "Map updated!" << std::endl;
    bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
    if(valid_jps)
    {
       std::cout << "JPS Planner succeeded!" << std::endl;
       std::cout << "error code: " << planner_ptr->status() << std::endl;
    }
    else
    {
        std::cout << "JPS Planner failed!" << std::endl;
    }
    const auto path_jps = planner_ptr->getRawPath(); // Get the planned raw path from JPS
    
    nav_msgs::Path path_msg = DecompROS::vec_to_path(path_jps);
    path_msg.header.frame_id = "odom";
    JPS_path_pub.publish(path_msg);
    return path_jps;
}

vec_Vec3f decomp_util::jps_planner_3d(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i)
{
    auto planner_ptr = std::make_shared<JPSPlanner3D>(true);  // Declare a planner
    Eigen::Vector3d start(start_sent(0), start_sent(1), std::max(start_sent(2), 0.0));
    Eigen::Vector3d goal(goal_sent(0), goal_sent(1), std::max(goal_sent(2), 0.0));

    Vec3f originalStart = start;

    pcl::PointXYZ pcl_start = pcl::PointXYZ(start(0), start(1), start(2));
    pcl::PointXYZ pcl_goal = pcl::PointXYZ(goal(0), goal(1), goal(2));

    ///////////////////////////////////////////////////////////////
    /////////////////////////// RUN JPS ///////////////////////////
    ///////////////////////////////////////////////////////////////

    

    // Set start and goal free
    const Veci<3> start_int = map_util_3d_->floatToInt(start);
    const Veci<3> goal_int = map_util_3d_->floatToInt(goal);

      map_util_3d_->setFreeVoxelAndSurroundings(start_int, 0.05);
      map_util_3d_->setFreeVoxelAndSurroundings(goal_int, 0.05);

    planner_ptr->setMapUtil(map_util_3d_);  // Set collision checking function
    planner_ptr->updateMap();
    std::cout << "Map updated!" << std::endl;
    bool valid_jps = planner_ptr->plan(start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and
                                                                // using JPS (if false --> use A*)

    vec_Vecf<3> path;
    path.clear();

    if (valid_jps == true)  // There is a solution
    {
        path = planner_ptr->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
        if (path.size() > 1)
        {
        path[0] = start;
        path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
        }
        else
        {  // happens when start and goal are very near (--> same cell)
        vec_Vecf<3> tmp;
        tmp.push_back(start);
        tmp.push_back(goal);
        path = tmp;
        }
    }
    else
    {
        std::cout << "JPS didn't find a solution from" << start.transpose() << " to " << goal.transpose() << std::endl;
    }

    *solved = valid_jps;
    nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "odom";
    JPS_3d_path_pub.publish(path_msg);
    return path;
}





