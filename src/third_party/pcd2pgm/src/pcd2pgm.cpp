#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/conditional_removal.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::OccupancyGrid map_topic_msg;
// minimum and maximum height
double thre_z_min = 0.3;
double thre_z_max = 2.0;
double map_resolution = 0.05;
int flag_pass_through = 0;
double thre_radius = 0.1;
// radius filter points threshold
int thres_point_count = 10;

pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_z(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_y(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough_x(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// pass through filter
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in);
// radius filter
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
// convert to grid map data and publish
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg);
void SetMapTopicMsgWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid &msg);
void FusionMaps(nav_msgs::OccupancyGrid &map1, nav_msgs::OccupancyGrid &map2, nav_msgs::OccupancyGrid &result_map);
void FilterMapMedian(nav_msgs::OccupancyGrid &map);
void FilterFusionMap(nav_msgs::OccupancyGrid &map);

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_filters");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Rate loop_rate(1.0);

  private_nh.param("file_directory", file_directory, std::string("/home/"));

  private_nh.param("file_name", file_name, std::string("map"));

  pcd_file = file_directory + file_name + pcd_format;

  private_nh.param("thre_z_min", thre_z_min, 0.2);
  private_nh.param("thre_z_max", thre_z_max, 2.0);
  private_nh.param("map_resolution", map_resolution, 0.05);
  private_nh.param("map_topic_name", map_topic_name, std::string("map"));

  ros::Publisher map_topic_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

  // load .pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    PCL_ERROR("Couldn't read file: %s \n", pcd_file.c_str());
    return (-1);
  }

  std::cout << "Initial pointcloud number : " << pcd_cloud->points.size() << std::endl;

  // pass through filter
  PassThroughFilter(thre_z_min, thre_z_max, static_cast<bool>(flag_pass_through));
  // radius filter
  RadiusOutlierFilter(cloud_after_PassThrough_x, thre_radius, thres_point_count);
  // convert to grid map data and publish
  // SetMapTopicMsg(cloud_after_Radius, map_topic_msg);
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto& point : cloud_after_Radius->points) {
      if (point.x < x_min) x_min = point.x;
      if (point.x > x_max) x_max = point.x;
      if (point.y < y_min) y_min = point.y;
      if (point.y > y_max) y_max = point.y;
  }

  std::cout << "cloud_after_Radius x range: [" << x_min << ", " << x_max << "]\n";
  std::cout << "cloud_after_Radius y range: [" << y_min << ", " << y_max << "]\n";

  pcl::PassThrough<pcl::PointXYZ> passthrough_x;
  passthrough_x.setInputCloud(pcd_cloud);
  passthrough_x.setFilterFieldName("x");
  passthrough_x.setFilterLimits(x_min, x_max);
  passthrough_x.filter(*cloud_after_PassThrough_x);

  pcl::PassThrough<pcl::PointXYZ> passthrough_y;
  passthrough_y.setInputCloud(cloud_after_PassThrough_x);
  passthrough_y.setFilterFieldName("y");
  passthrough_y.setFilterLimits(y_min, y_max);
  passthrough_y.filter(*pcd_cloud);  

  nav_msgs::OccupancyGrid map1;
  SetMapTopicMsgWithPlane(pcd_cloud, map1);
  FilterMapMedian(map1);

  nav_msgs::OccupancyGrid map2;
  SetMapTopicMsg(cloud_after_Radius, map2);

  nav_msgs::OccupancyGrid fused_map;
  FusionMaps(map1, map2, fused_map);
  FilterFusionMap(fused_map);

  while (ros::ok()) {
    map_topic_pub.publish(fused_map);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}

// filter pointcloud using pass through
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {

  // create filter_z
  pcl::PassThrough<pcl::PointXYZ> passthrough_z;
  // input pointcloud
  passthrough_z.setInputCloud(pcd_cloud);
  // set operation in z axis
  passthrough_z.setFilterFieldName("z");
  // set height range
  passthrough_z.setFilterLimits(thre_low, thre_high);
  // true : keep points out of range / false : keep points in the range
  passthrough_z.setFilterLimitsNegative(flag_in);
  // do filtering and save
  passthrough_z.filter(*cloud_after_PassThrough_z);

  // create filter_y
  pcl::PassThrough<pcl::PointXYZ> passthrough_y;
  // input pointcloud
  passthrough_y.setInputCloud(cloud_after_PassThrough_z);
  // set operation in y axis
  passthrough_y.setFilterFieldName("y");
  // set height range
  passthrough_y.setFilterLimits(-100, 100);
  // true : keep points out of range / false : keep points in the range
  passthrough_y.setFilterLimitsNegative(false);
  // do filtering and save
  passthrough_y.filter(*cloud_after_PassThrough_y);

  // create filter_x
  pcl::PassThrough<pcl::PointXYZ> passthrough_x;
  // input pointcloud
  passthrough_x.setInputCloud(cloud_after_PassThrough_y);
  // set operation in x axis
  passthrough_x.setFilterFieldName("x");
  // set height range
  passthrough_x.setFilterLimits(-100, 100);
  // true : keep points out of range / false : keep points in the range
  passthrough_x.setFilterLimitsNegative(false);
  // do filtering and save
  passthrough_x.filter(*cloud_after_PassThrough_x);

  // save to pcd file
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd",
                                      *cloud_after_PassThrough_x);
  std::cout << "pass through filter pointcloud : "
            << cloud_after_PassThrough_x->points.size() << std::endl;
}

// radius filter
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  // create filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  // define input pointcloud
  radiusoutlier.setInputCloud(pcd_cloud0);
  // set radius and find point in range
  radiusoutlier.setRadiusSearch(radius);
  // delete points if < threshold
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  // save to pcd file
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                      *cloud_after_Radius);
  std::cout << "radius filter pointcloud : " << cloud_after_Radius->points.size()
            << std::endl;
}
void FusionMaps(nav_msgs::OccupancyGrid &map1, nav_msgs::OccupancyGrid &map2, nav_msgs::OccupancyGrid &result_map) {
    if (map1.info.width != map2.info.width || map1.info.height != map2.info.height) {
        ROS_ERROR("Map sizes do not match.");
        return;
    }

    result_map = map1;
    result_map.data.resize(map1.info.width * map1.info.height);

    for (int i = 0; i < map1.data.size(); ++i) {
        if (map1.data[i] == 100 || map2.data[i] == 100) {
            result_map.data[i] = 100;
        } else {
            result_map.data[i] = 0; 
        }
    }
}

void FilterMapMedian(nav_msgs::OccupancyGrid &map) {
    int width = map.info.width;
    int height = map.info.height;
    std::vector<int8_t> filtered_data = map.data;

    int filter_size = 3;
    int offset = filter_size / 2;

    for (int r = offset; r < height - offset; ++r) {
        for (int c = offset; c < width - offset; ++c) {
            std::vector<int> window;
            for (int dr = -offset; dr <= offset; ++dr) {
                for (int dc = -offset; dc <= offset; ++dc) {
                    int idx = (r + dr) * width + (c + dc);
                    window.push_back(static_cast<int>(map.data[idx]));
                }
            }
            std::sort(window.begin(), window.end());
            int median = window[window.size() / 2];
            filtered_data[r * width + c] = static_cast<int8_t>(median);
        }
    }

    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    int min_region_size = 5000;  

    auto bfs = [&](int start_row, int start_col) {
        std::vector<std::pair<int, int>> region;
        std::queue<std::pair<int, int>> q;
        q.push({start_row, start_col});
        visited[start_row][start_col] = true;
        region.push_back({start_row, start_col});
        
        const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        
        while (!q.empty()) {
            auto [row, col] = q.front();
            q.pop();

            for (const auto& dir : directions) {
                int new_row = row + dir.first;
                int new_col = col + dir.second;

                if (new_row >= 0 && new_row < height && new_col >= 0 && new_col < width &&
                    !visited[new_row][new_col] && filtered_data[new_row * width + new_col] == 100) {
                    visited[new_row][new_col] = true;
                    q.push({new_row, new_col});
                    region.push_back({new_row, new_col});
                }
            }
        }
        return region;
    };

    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            if (!visited[r][c] && filtered_data[r * width + c] == 100) {

                std::vector<std::pair<int, int>> region = bfs(r, c);

                if (region.size() < min_region_size) {
                    for (const auto& [row, col] : region) {
                        filtered_data[row * width + col] = 0; // 删除小区域，设为 0
                    }
                }
            }
        }
    }

    map.data = filtered_data;
}

void FilterFusionMap(nav_msgs::OccupancyGrid &map) {
    int width = map.info.width;
    int height = map.info.height;
    std::vector<int8_t> filtered_data = map.data;

    // Set filter window size
    int filter_size = 1;
    int offset = filter_size / 2;

    // Apply median filtering to the map, skipping the borders
    for (int r = offset; r < height - offset; ++r) {
        for (int c = offset; c < width - offset; ++c) {
            std::vector<int> window;
            // Collect values in the filter_size x filter_size neighborhood
            for (int dr = -offset; dr <= offset; ++dr) {
                for (int dc = -offset; dc <= offset; ++dc) {
                    int idx = (r + dr) * width + (c + dc);
                    window.push_back(static_cast<int>(map.data[idx]));
                }
            }
            // Sort the values in the window and take the median
            std::sort(window.begin(), window.end());
            int median = window[window.size() / 2];
            filtered_data[r * width + c] = static_cast<int8_t>(median);
        }
    }

    // Update the map data with the filtered result
    map.data = filtered_data;
}


void SetMapTopicMsgWithPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid &msg) {
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;

    if (cloud->points.empty()) {
        ROS_WARN("pcd is empty!\n");
        return;
    }

    for (int i = 0; i < cloud->points.size() - 1; i++) {
        if (i == 0) {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;
    }

    // define origin position
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    // define grid map size
    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);

    // point coord (x,y) in real map corresponding to grid map coord [x*map.info.width+y]
    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 100);

    ROS_INFO("data size = %d\n", msg.data.size());

    for (int iter = 0; iter < cloud->points.size(); iter++) {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= msg.info.width)
            continue;

        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= msg.info.height - 1)
            continue;

        // grid map's Occupancy posibility [0,100]
        msg.data[i + j * msg.info.width] = 0;
    }
}

// convert to grid map data and publish
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg) {
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;

  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for (int i = 0; i < cloud->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // define origin position
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  // define grid map size
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  // point coord (x,y) in real map corresponding to grid map coord [x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for (int iter = 0; iter < cloud->points.size(); iter++) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // grid map's Occupancy posibility [0,100]
    msg.data[i + j * msg.info.width] = 100;
  }
}
