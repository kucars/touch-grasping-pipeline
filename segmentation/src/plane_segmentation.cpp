#include <ros/ros.h>
#include <iostream>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

#include <segmentation/Table.h>
#include <visualization_msgs/Marker.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "marker_detect");
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_("~");
  
  int inlier_threshold_;
  tf::TransformListener listener_;
  std::string processing_frame_;
  double plane_detection_voxel_size_;
  double clustering_voxel_size_;
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  double table_z_filter_min_, table_z_filter_max_;
  double cluster_distance_;
  int min_cluster_size_;
  double up_direction_;
  bool flatten_table_;
  double table_padding_;
  
  priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
  priv_nh_.param<std::string>("processing_frame", processing_frame_, "base_link");
  priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
  priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
  priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.0);
  priv_nh_.param<double>("z_filter_max", z_filter_max_, 0.0);
  priv_nh_.param<double>("y_filter_min", y_filter_min_, -0.5);
  priv_nh_.param<double>("y_filter_max", y_filter_max_, 0.5);
  priv_nh_.param<double>("x_filter_min", x_filter_min_, 0.0);
  priv_nh_.param<double>("x_filter_max", x_filter_max_, 0.5);
  priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
  priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.50);
  priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
  priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
  //NOT NEEDED 
  priv_nh_.param<double>("up_direction", up_direction_, -1.0);
  priv_nh_.param<bool>("flatten_table", flatten_table_, false);
  priv_nh_.param<double>("table_padding", table_padding_, 0.0);
  
  //from the tutorial
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  
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
 
    typedef pcl::PointXYZRGB    Point;
    typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;
   
      // PCL objects
    KdTreePtr normals_tree_, clusters_tree_;
    pcl::VoxelGrid<Point> grid_, grid_objects_;
    pcl::PassThrough<Point> pass_;
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
    pcl::ProjectInliers<Point> proj_;
    pcl::ConvexHull<Point> hull_;
    pcl::ExtractPolygonalPrismData<Point> prism_;
    pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
    pcl::PointCloud<Point>::Ptr table_hull_ptr (new pcl::PointCloud<Point>);
   
     // Filtering parameters
    grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
    grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
    grid_.setFilterFieldName ("z");
    grid_.setFilterLimits (x_filter_min_,  x_filter_max_);
    grid_.setDownsampleAllData (false);
    grid_objects_.setDownsampleAllData (true);
    
    normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
    clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
    
    // Normal estimation parameters
    n3d_.setKSearch (10);
    n3d_.setSearchMethod (normals_tree_);
    // Table model fitting parameters
    seg_.setDistanceThreshold (0.05);
    seg_.setMaxIterations (10000);
    seg_.setNormalDistanceWeight (0.1);
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setProbability (0.99);

    proj_.setModelType (pcl::SACMODEL_PLANE);
   
    
       // Clustering parameters
    pcl_cluster_.setClusterTolerance (cluster_distance_);
    pcl_cluster_.setMinClusterSize (min_cluster_size_);
    pcl_cluster_.setSearchMethod (clusters_tree_);
    
    
     // Step 1 : Filter, remove NaNs and downsample
    pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>);
    try
    {
        pcl::fromROSMsg (converted_cloud, *cloud_ptr);
    }
    catch (...)
    {
        ROS_ERROR("Failure while converting the ROS Message to PCL point cloud.  Tabletop segmentation now requires the input cloud to have color info, so you must input a cloud with XYZRGB points, not just XYZ.");
        throw;
    }
      // save the cloud to PCD file //****************************************
   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZRGB> ("input_cloud.pcd", *cloud_ptr, false);
    //***********************************************************************************************
    
//     pass_.setInputCloud (cloud_ptr);
//     pass_.setFilterFieldName ("z");
//     pass_.setFilterLimits (z_filter_min_, z_filter_max_);
//     pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl::PointCloud<Point>);
//     pass_.filter (*z_cloud_filtered_ptr);
// 
//     pass_.setInputCloud (z_cloud_filtered_ptr);
//     pass_.setFilterFieldName ("y");
//     pass_.setFilterLimits (y_filter_min_, y_filter_max_);
//     pcl::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl::PointCloud<Point>);
//     pass_.filter (*y_cloud_filtered_ptr);
// 
//     pass_.setInputCloud (y_cloud_filtered_ptr);
//     pass_.setFilterFieldName ("x");
//     pass_.setFilterLimits (x_filter_min_, x_filter_max_);
//     pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>);
//     pass_.filter (*cloud_filtered_ptr);
    
    //**********code from the filtering tutorial***************************************
    pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_ptr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered_ptr);
    //**********************************************************************************
    ROS_INFO("Step 1 done");
    if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_)
    {
        ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
    }
    // save the filtered cloud to PCD file //****************************************
    //pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("cloud_filtered3.pcd", *cloud_filtered_ptr, false);
    //***********************************************************************************************

    pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<Point>);
    grid_.setInputCloud (cloud_filtered_ptr);
    grid_.filter (*cloud_downsampled_ptr);
    if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_)
    {
        ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
    }
    // save the downsampled cloud to PCD file //****************************************
   // pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("cloud_downsampled3.pcd", *cloud_downsampled_ptr, false);
    //***********************************************************************************************
    
    // Step 2 : Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
    n3d_.setInputCloud (cloud_downsampled_ptr);
    n3d_.compute (*cloud_normals_ptr);
    ROS_INFO("Step 2 done");

    
    // Step 3 : Perform planar segmentation, if table is not given, otherwise use given table
    tf::Transform table_plane_trans;//not sure if they are needed
    tf::Transform table_plane_trans_flat;
    
    
    pcl::PointIndices::Ptr table_inliers_ptr (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr table_coefficients_ptr (new pcl::ModelCoefficients);
    seg_.setInputCloud (cloud_downsampled_ptr);
    seg_.setInputNormals (cloud_normals_ptr);
    seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);
    
     if (table_coefficients_ptr->values.size () <=3)
        {
            ROS_INFO("Failed to detect table in scan");
        }

        if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_)
        {
            ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(),inlier_threshold_);
        }

        ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].",
                  (int)table_inliers_ptr->indices.size (),
                  table_coefficients_ptr->values[0], table_coefficients_ptr->values[1],
                  table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
        ROS_INFO("Step 3 done");
    
    
     // Step 4 : Project the table inliers on the table
    pcl::PointCloud<Point>::Ptr table_projected_ptr (new pcl::PointCloud<Point>);
    proj_.setInputCloud (cloud_downsampled_ptr);
    proj_.setIndices (table_inliers_ptr);
    proj_.setModelCoefficients (table_coefficients_ptr);
    proj_.filter (*table_projected_ptr);
    ROS_INFO("Step 4 done");
        // save the projected inlier to PCD file //****************************************
    //pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("table_projected3.pcd", *table_projected_ptr, false);
    //***********************************************************************************************
    
    pcl::visualization::CloudViewer viewer ("viewer");
    viewer.showCloud (table_projected_ptr);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }


  return (0);
}
