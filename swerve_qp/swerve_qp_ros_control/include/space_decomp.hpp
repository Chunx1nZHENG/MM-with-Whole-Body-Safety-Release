#pragma once
#include <decomp_ros_utils/data_ros_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <jps_planner/jps_planner/jps_planner.h>
#include <geometry_msgs/PointStamped.h>
using Vec3f = Eigen::Matrix<double , 3, 1>;

using vec_Vec3f = std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>>;
using JPS::OccMapUtil;
using JPS::VoxelMapUtil;


// judge if a point is in a circle
template <typename VecT>
bool isPointInCircle(const VecT& point, const VecT& circle_center, double radius) {
    double distance = (point - circle_center).norm();
    return distance <= radius;
}


template <typename VecT>
VecT interpolate(const VecT& seg_start, const VecT& seg_end, const VecT& center, double radius, int& state) {
    VecT direction = seg_end - seg_start;
    VecT seg_vector = direction.normalized();

    // find the closest point on the segment to the circle (or sphere) center
    VecT to_center = center - seg_start;
    double projection_length = to_center.dot(seg_vector);
    VecT closest_point = seg_start + projection_length * seg_vector;

    double dist_to_center = (closest_point - center).norm();

    // if the closest point is outside the circle/sphere, return the closest point on the circle/sphere
    if (dist_to_center >= radius) {
        std::cerr << "No intersection." << std::endl;
        state = 0;
        if ((closest_point - seg_start).dot(direction) < 0) {
            return seg_start;
        }
        else if ((closest_point - seg_end).dot(direction) > 0) {
            return seg_end;
        }
        else {
            return closest_point;
        }
    }

    double offset_length = sqrt(radius * radius - dist_to_center * dist_to_center);
    VecT intersection1 = closest_point + offset_length * seg_vector;
    VecT intersection2 = closest_point - offset_length * seg_vector;

    // Check if the intersection points are within the segment bounds
    if ((intersection1 - seg_start).dot(direction) >= 0 && (intersection1 - seg_end).dot(direction) <= 0  && (intersection1 - closest_point).dot(direction) >= 0) {
        state = 1;
        return intersection1;
    } else if ((intersection2 - seg_start).dot(direction) >= 0 && (intersection2 - seg_end).dot(direction) <= 0 && (intersection2 - closest_point).dot(direction) >= 0) {
        state = 1;
        return intersection2;
    } else {
        state = 2;
        if ((closest_point - seg_start).dot(direction) < 0) {
            return seg_start;
        }
        else if ((closest_point - seg_end).dot(direction) > 0) {
            return seg_end;
        }
        else {
            return closest_point;
        }
    }
}


class decomp_util{

    public:
    decomp_util(){};
    decomp_util(ros::NodeHandle nh);
    pcl::PointCloud<pcl::PointXYZ> readPCDFile(const std::string& file_name);
    void decomp_and_publish(vec_Vec3f path, pcl::PointCloud<pcl::PointXYZ>& cloud, double radiu);
    void decomp_and_publish(vec_Vec3f path, pcl::PointCloud<pcl::PointXYZ>& cloud);
    std::vector<Vec3f> getFreeRegionCenters(){return freeregionCenters;};
    std::vector<LinearConstraint3D> getLinearConstraints(){return LinearConstraints_;};
    vec_Vec2f jps_planner(Vec2f start, Vec2f goal );
    
    vec_Vec3f jps_planner_3d(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i);
    
    template <typename VecT>
    VecT get_goal_from_path(const vec_E<VecT> &path, VecT start, double follow_radiu, double ee_radiu)
    {
        VecT goal;
        int state = 0;
        geometry_msgs::PointStamped base_goal;
        base_goal.header.frame_id = "odom";
        base_goal.header.stamp = ros::Time::now();
        
        // Path empty check
        if (path.empty()) {
            goal = start;
            std::cerr << "Path is empty!" << std::endl;
            base_goal.point.x = goal[0];
            base_goal.point.y = goal[1];
            if constexpr (std::is_same<VecT, Vec3f>::value) {
                base_goal.point.z = goal[2];
                ee_goal_pub.publish(base_goal);
            }
            else {
                base_goal_pub.publish(base_goal);
            }
            return goal;
        }
        VecT closest_point = path.back();
        double min_dist = (start - path.back()).norm();  
        // Find the closest point on the path to the start point
        int pt_ind = 0;
        for (int i = 0; i < path.size()-1; i++) {
            auto t_goal = interpolate(path[i], path[i + 1], start, follow_radiu, state);
                if (state == 1) {
                    goal = t_goal;
                    break;
                }
                else
                {
                    double dist = (start - t_goal).norm();
                    if (dist < min_dist) {
                        min_dist = dist;
                        closest_point = t_goal;
                        pt_ind = i;
                    }
                }
        }
        // If within end-effector radius, adjust the goal accordingly
        if (state == 1) {
            if (isPointInCircle(goal, path.back(), ee_radiu)) {
                int t_state;
                // if (isPointInCircle(goal, path.back(), ee_radiu) && std::is_same<VecT, Vec3f>::value) {
                if (isPointInCircle(goal, path.back(), ee_radiu) ) {
                    goal = path.back();
                }
                else
                {
                    goal = interpolate(path.back(), path[path.size() - 2], path.back(), ee_radiu, t_state);
                    if (t_state != 1) {
                        goal = path[path.size() - 2];
                    }
                }
                
            }
        }
        else {
            goal = closest_point;
        //    if (isPointInCircle(goal, path.back(), ee_radiu) && std::is_same<VecT, Vec3f>::value) {
        if (isPointInCircle(goal, path.back(), ee_radiu)) {
                    goal = path.back();
                }
        }

        // Publish the goal
        base_goal.point.x = goal[0];
        base_goal.point.y = goal[1];
        if constexpr (std::is_same<VecT, Vec3f>::value) {
            base_goal.point.z = goal[2];
            ee_goal_pub.publish(base_goal);
        }
        else {
            base_goal_pub.publish(base_goal);
        }
        

        return goal;
    }

    std::shared_ptr<OccMapUtil>   map_util_;
    std::shared_ptr<VoxelMapUtil>  map_util_3d_;
    bool map_util_first_set = false;
    double map_resolution_;
    std::vector<double> map_size_;
    ros::Publisher ee_goal_pub;

    private:
    ros::Publisher cloud_pub ;
    ros::Publisher path_pub ;
    ros::Publisher es_pub ;
    ros::Publisher poly_pub ;
    ros::Publisher GridMap_pub ;
    ros::Publisher JPS_path_pub;
    ros::Publisher JPS_3d_path_pub;
    ros::Publisher Octomap_pub;
    ros::Publisher base_goal_pub;
    
    ros::NodeHandle nh_;
    std::vector<LinearConstraint3D> LinearConstraints_;
    std::vector<Vec3f> freeregionCenters;
    std::vector<int8_t> gridData_ ;
    
    
};



