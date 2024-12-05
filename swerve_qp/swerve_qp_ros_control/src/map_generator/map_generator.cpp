#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZ PointT;

//INPUT left_top_x, right_bottom_x, robot_radius, map_resolution
void GenerateBaseMap(double left_top_x, double right_bottom_x, double robot_radius, double map_resolution){
    //initialize a cloud of points
    pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>);
    //find a point randomly in map
    double x = left_top_x + (right_bottom_x - left_top_x) * rand() / (RAND_MAX + 1.0);
    double y = left_top_x + (right_bottom_x - left_top_x) * rand() / (RAND_MAX + 1.0);
    double z = 0.1;
    //add the point to the points
    PointT point;
    point.x = x;
    point.y = y;
    point.z = z;
    points->push_back(point);

    //From the last point, generate a new point, the distance between the new point and the last point is equal to robot_radius*2
    //The new point should be in the range of the map, if can not find point for 100 times, then stop
    int count = 0;
    while (count < 100){
        double new_x = x + robot_radius * 2 * (rand() % 2);
        double new_y = y + robot_radius * 2 * (rand() % 2);
        if (new_x >= left_top_x && new_x <= right_bottom_x && new_y >= left_top_x && new_y <= right_bottom_x){
            PointT new_point;
            new_point.x = new_x;
            new_point.y = new_y;
            new_point.z = z;
            points->push_back(new_point);
            x = new_x;
            y = new_y;
            count = 0;
        }
        else{
            count++;
        }
    }
    //print the points
    for (int i = 0; i < points->size(); i++){
        std::cout << "x: " << points->points[i].x << " y: " << points->points[i].y << " z: " << points->points[i].z << std::endl;
    }
}

int main(){
    GenerateBaseMap(0, 10, 0.5, 0.1);
    return 0;
}