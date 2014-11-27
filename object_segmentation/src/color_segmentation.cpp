#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

 #include <vector>
 #include <pcl/search/search.h>
 #include <pcl/segmentation/region_growing_rgb.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



int main (int argc, char** argv)
{
  
  ros::init (argc, argv, "color_segmentation");
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  
  typedef pcl::PointXYZRGB Point;
  
  std::string processing_frame_="base_link";
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
 
  
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  ROS_INFO(" waiting for a point_cloud2 on topic %s", topic.c_str());
  
  sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic);
  sensor_msgs::PointCloud old_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
        int current_try=0, max_tries = 3;
        while (1)
        {
            bool transform_success = true;
            try
            {
                listener_.transformPointCloud(processing_frame_, old_cloud, old_cloud);
            }
            catch (tf::TransformException ex)
            {
                transform_success = false;
                if (++current_try >= max_tries)
                {
                    ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)", old_cloud.header.frame_id.c_str(),
                              processing_frame_.c_str(), current_try);
                }
                ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
                //sleep a bit to give the listener a chance to get a new transform
                ros::Duration(0.1).sleep();
            }
            if (transform_success) break;
        }
   
   sensor_msgs::PointCloud2 converted_cloud;
   sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
   pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  // pcl::PointCloud <pcl::Point>::Ptr cloud (new pcl::PointCloud <pcl::Point>);
    try
    {
        pcl::fromROSMsg(converted_cloud, *cloud);
    }
    catch (...)
    {
        ROS_ERROR("Failure while converting the ROS Message to PCL point cloud.  segmentation now requires the input cloud to have color info, so you must input a cloud with XYZRGB points, not just XYZ.");
        throw;
    }
  
  
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters.size ());

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
  

  return (0);
}