#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/registration/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/boost.h>
#include <pcl/tracking/particle_filter.h>
#include <Eigen/Dense>
//#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <math.h>
#include <algorithm>
#include <vector>
template <typename PointT> void
getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud_objects, 			    
                            const std::vector<pcl::PointIndices> &clusters2,
                            std::vector<sensor_msgs::PointCloud> &clusters)
{
    clusters.resize (clusters2.size ());
    for (size_t i = 0; i < clusters2.size (); ++i)
    {
        pcl::PointCloud<PointT> cloud_cluster;
        pcl::copyPointCloud(cloud_objects, clusters2[i], cloud_cluster);
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg( cloud_cluster, pc2 );
        sensor_msgs::convertPointCloud2ToPointCloud (pc2, clusters[i]);
    }
}
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction, bool flatten_plane)
{
    ROS_ASSERT(coeffs.values.size() > 3);
    double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
    //asume plane coefficients are normalized
    tf::Vector3 position(-a*d, -b*d, -c*d);
    tf::Vector3 z(a, b, c);

    //if we are flattening the plane, make z just be (0,0,up_direction)
    if(flatten_plane)
    {
        ROS_INFO("flattening plane");
        z[0] = z[1] = 0;
        z[2] = up_direction;
    }
    else
    {
        //make sure z points "up"
        ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
        if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
        {
            z = -1.0 * z;
            ROS_INFO("flipped z");
        }
    }
    
    //try to align the x axis with the x axis of the original frame
    //or the y axis if z and x are too close too each other
    tf::Vector3 x(1, 0, 0);
    if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
    tf::Vector3 y = z.cross(x).normalized();
    x = y.cross(z).normalized();

    tf::Matrix3x3 rotation;
    rotation[0] = x; 	// x
    rotation[1] = y; 	// y
    rotation[2] = z; 	// z
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);
    ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
    ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
                     const tf::Transform& table_plane_trans,
                     sensor_msgs::PointCloud &table_points)
{
    // Prepare the output
    table_points.header = pcl_conversions::fromPCL(table.header);
    // table_points.header = table.header;
    table_points.points.resize (table.points.size ());
    for (size_t i = 0; i < table.points.size (); ++i)
    {
        table_points.points[i].x = table.points[i].x;
        table_points.points[i].y = table.points[i].y;
        table_points.points[i].z = table.points[i].z;
    }

    // Transform the data
    tf::TransformListener listener;
    tf::StampedTransform table_pose_frame(table_plane_trans, table_points.header.stamp,
                                          table.header.frame_id, "table_frame");
    listener.setTransform(table_pose_frame);
    std::string error_msg;
    if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
    {
        ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
                  table_points.header.frame_id.c_str(), error_msg.c_str());
        return false;
    }
    int current_try=0, max_tries = 3;
    while (1)
    {
        bool transform_success = true;
        try
        {
            listener.transformPointCloud("table_frame", table_points, table_points);
        }
        catch (tf::TransformException ex)
        {
            transform_success = false;
            if ( ++current_try >= max_tries )
            {
                ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
                          table_points.header.frame_id.c_str(), ex.what());
                return false;
            }
            //sleep a bit to give the listener a chance to get a new transform
            ros::Duration(0.1).sleep();
        }
        if (transform_success) break;
    }
    table_points.header = pcl_conversions::fromPCL(table.header);
    //table_points.header.stamp = table.header.stamp;
    table_points.header.frame_id = "table_frame";
    return true;
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "point");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    //    ros::Publisher position_pub = nh.advertise<geometry_msgs::Pose>("visualization_marker", 50);
    // ros::Publisher point_pub = nh.advertise<geometry_msgs::Pose>("", 10);


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

    priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 500);
    priv_nh_.param<std::string>("processing_frame", processing_frame_, "base_link");
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, -0.1);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, 0.5);
    priv_nh_.param<double>("y_filter_min", y_filter_min_, -0.4);
    priv_nh_.param<double>("y_filter_max", y_filter_max_, 0.4);
    priv_nh_.param<double>("x_filter_min", x_filter_min_, 0.0);
    priv_nh_.param<double>("x_filter_max", x_filter_max_, 0.5);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, -0.5);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, -0.01);
    priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.08);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 150);
    //NOT NEEDED
    priv_nh_.param<double>("up_direction", up_direction_, 1.0);
    priv_nh_.param<bool>("flatten_table", flatten_table_, false);
    priv_nh_.param<double>("table_padding", table_padding_, 0.0);
    //Marker for visualization
    visualization_msgs::Marker marker;
    ros::Rate r(1);
    while(ros::ok())
    {

        ros::Time begin = ros::Time::now();
        //from the tutorial
        std::string topic = nh.resolveName("/camera/depth_registered/points");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic);
        sensor_msgs::PointCloud old_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
        int current_try=0, max_tries = 3;
        while (1)
        {
            bool transform_success = true;
            try
            {
                listener_.transformPointCloud("/base_link", old_cloud, old_cloud);
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
            if (transform_success)
                break;
        }

        sensor_msgs::PointCloud2 converted_cloud;
        sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);

        // Read in the cloud data***************************************
        //pcl::PCDReader reader;
        // pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>), cloud_f (new pcl::PointCloud<Point>);
        //reader.read ("table_scene_lms400.pcd", *cloud);
        //reader.read ("cloud_cluster_0.pcd", *cloud);
        // ****************************************************************



        //********************************************** ***********************************************************************************************
        //************************plane segmentation ***********************************************************************************************

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
        pcl_cluster_.setClusterTolerance (0.05);
        pcl_cluster_.setMinClusterSize (2900);
        pcl_cluster_.setMaxClusterSize (3300);
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
        //writer.write<pcl::PointXYZRGB> ("input_cloud.pcd", *cloud_ptr, false);
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
        //writer.write<pcl::PointXYZRGB> ("cloud_filtered3.pcd", *cloud_filtered_ptr, false);
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
        //writer.write<pcl::PointXYZRGB> ("cloud_downsampled3.pcd", *cloud_downsampled_ptr, false);
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
        //writer.write<pcl::PointXYZRGB> ("/home/seekursdp/catkin_ws_independent/table_projected4.pcd", *table_projected_ptr, false);
        //***********************************************************************************************

        //     pcl::visualization::CloudViewer viewer ("viewer");
        //     viewer.showCloud (table_projected_ptr);
        //     while (!viewer.wasStopped ())
        //     {
        //       boost::this_thread::sleep (boost::posix_time::microseconds (100));
        //     }
        //
        //***********************************************************************************************************************
        //******************************plane segmentaiton end*******************************************************************




        //***********************************************************************************************************************
        //***********************************************clusters extraction *******************************************************
        sensor_msgs::PointCloud table_points;
        sensor_msgs::PointCloud table_hull_points;
        table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);

        // ---[ Estimate the convex hull (not in table frame)
        hull_.setInputCloud (table_projected_ptr);
        hull_.reconstruct (*table_hull_ptr);


        // Step 5: Get the objects on top of the (non-flat) table
        pcl::PointIndices cloud_object_indices;
        //prism_.setInputCloud (cloud_all_minus_table_ptr);
        prism_.setInputCloud (cloud_filtered_ptr);
        prism_.setInputPlanarHull (table_hull_ptr);
        ROS_INFO("Using table prism: %f to %f", table_z_filter_min_, table_z_filter_max_);
        prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);
        prism_.segment (cloud_object_indices);

        pcl::PointCloud<Point>::Ptr cloud_objects_ptr (new pcl::PointCloud<Point>);
        pcl::ExtractIndices<Point> extract_object_indices;
        extract_object_indices.setInputCloud (cloud_filtered_ptr);
        extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
        extract_object_indices.filter (*cloud_objects_ptr);

        ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects_ptr->points.size ());

        if (cloud_objects_ptr->points.empty ())
        {
            ROS_INFO("No objects on table");
            return 0;
        }

        //*****************************************Before Downsampling ****************************************
        //pcl::PCDWriter writer;
        //writer.write<pcl::PointXYZRGB> ("/home/seekursdp/catkin_ws_independent/cloud_objects_ptr.pcd", *cloud_objects_ptr, false);
        ROS_INFO (" Done writing to PCD file before downsampling ");
        //***************************************************************************************************


        // ---[ Downsample the points
        pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>);
        grid_objects_.setInputCloud (cloud_objects_ptr);
        grid_objects_.filter (*cloud_objects_downsampled_ptr);


        //*****************************************after Downsampling ****************************************
        pcl::PCDWriter writer2;
        //writer2.write<pcl::PointXYZRGB> ("/home/seekursdp/catkin_ws_independent/cloud_objects_downsampled.pcd", *cloud_objects_downsampled_ptr, false);
        ROS_INFO (" Done writing to PCD file after downsampling ");
        //***************************************************************************************************

        // Step 6: Split the objects into Euclidean clusters
        std::vector<pcl::PointIndices> clusters2;
        //pcl_cluster_.setInputCloud (cloud_objects_ptr);
        pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
        pcl_cluster_.extract (clusters2);
        ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

        // ---[ Convert clusters into the PointCloud message
        std::vector<sensor_msgs::PointCloud> clusters;
        getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters);

        //*****************************************after clustering ********************************************************
        //TODO:: REMOVE ME!!!
        int j = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<pcl::PointIndices>::const_iterator it = clusters2.begin (); it != clusters2.end (); ++it)
        {
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (cloud_objects_downsampled_ptr->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            j++;
        }
        //*****************************************after clustering ********************************************************


        //*******************************************************surface normal estimation ************************************************
        //  ax+by+cz+d=0
        std::cerr << "Plane coefficients: " <<table_coefficients_ptr->values[0]<< " "
                  << table_coefficients_ptr->values[1] << " "
                  << table_coefficients_ptr->values[2] << " "
                  << table_coefficients_ptr->values[3] << std::endl;
        //     //plane normal
        pcl::Normal plane_normal ((float)table_coefficients_ptr->values[0], (float)table_coefficients_ptr->values[1], (float)table_coefficients_ptr->values[2]);

        pcl::NormalEstimation<Point, pcl::Normal> ne;

        ne.setInputCloud (cloud_cluster);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        //pcl::search::Search <pcl::PointXYZRGB>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod (tree2);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);
        //        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        //        viewer.setBackgroundColor (0.0, 0.0, 0.5);
        //        viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_cluster, cloud_normals);

        //        while (!viewer.wasStopped ())
        //        {
        //            viewer.spinOnce ();
        //        }
        //**********************************************************point evaluation **********************************************************
        // now we have the plane normal and the cloud normals (object)
        // just for testing
        // pcl::Normal plane ((float)cloud_normals->points.at(i).normal_x, (float)cloud_normals->points.at(i).normal_y, (float)cloud_normals->points.at(i).normal_z);

        Eigen::Vector4f p;
        Eigen::Vector4f plane;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*cloud_cluster, centroid); //compute the centroid of the object (cluster)

        std::cout<<" centroid x: " <<centroid[0]<<std::endl;
        std::cout<<" centroid y: " <<centroid[1]<<std::endl;
        std::cout<<" centroid z: " <<centroid[2]<<std::endl;


        //Eigen Plane normal point
        plane[0]=plane_normal.normal_x;
        plane[1]=plane_normal.normal_y;
        plane[2]=plane_normal.normal_z;

        std::vector<double> angles;
        std::vector<double> distances;
        std::vector<double> cen_distances;
        std::vector<double> h;

        std::cout<<" width22" <<cloud_cluster->points.size()<<std::endl;

        const double pi= std::acos(-1);

        for (int i=0; i<cloud_normals->points.size () ; i++)
        {
            //computing the angles
            p[0]=cloud_normals->points.at(i).normal_x;
            p[1]=cloud_normals->points.at(i).normal_y;
            p[2]=cloud_normals->points.at(i).normal_z;
            //  angles[i]=pcl::getAngle3D(plane_normal, Eigen::Map<Eigen::Vector4f> tmp(cloud_normals->points.at(i).normal););
            angles.push_back(pcl::getAngle3D(p, plane));
            //   std::cout << "angle " <<i<<" "<< angles[i]<<std::endl;

            //computing the distance between two normals (the surface normal and the plane )
            // Ref: http://mathinsight.org/distance_point_plane
            double distance = fabs(plane[0]*cloud_cluster->points[i].x + plane[1]*cloud_cluster->points[i].y + plane[2]*cloud_cluster->points[i].z + plane[3])/sqrt(plane[0]*plane[0]+plane[1]*plane[1]+plane[2]*plane[2]);
            distances.push_back(distance);
            //            std::cout << "distance " <<i<<" "<< distances[i]<<std::endl;
            //            std::cout << "cloud_cluster x:" <<cloud_cluster->points[i].x <<" y:"<< cloud_cluster->points[i].y<<" z:"<<cloud_cluster->points[i].z<<std::endl;
            //            std::cout << "Plane Normal x:" <<plane[0] <<" y:"<< plane[1]<<" z:"<<plane[2]<<std::endl;

            //compute the distance from the center of the normals to the centroid of the object
            //    std::cout<<" vector x " <<p[0]<<" vector y " <<p[1]<<" vector z " <<p[2]<<std::endl;

            double dist2Centroid = sqrt((centroid[0]-cloud_cluster->points[i].x)*(centroid[0]-cloud_cluster->points[i].x) + (centroid[1]-cloud_cluster->points[i].y)*(centroid[1]-cloud_cluster->points[i].y) + (centroid[2]-cloud_cluster->points[i].z)*(centroid[2]-cloud_cluster->points[i].z));
            cen_distances.push_back(dist2Centroid);
            //            std::cout << "Eclidean distance from centroid " <<i<<" "<<dist2Centroid<<std::endl;

        }


        //finding the largest angles and distance from the arrays
        double aLower, aUpper;
        aLower = *min_element(angles.begin(), angles.end());
        aUpper = *max_element(angles.begin(), angles.end());
        std::cout << "Angle maximum value " <<aUpper<<std::endl;
        std::cout << "Angle minimum value " <<aLower<<std::endl;

        double dLower, dUpper;
        dLower = *min_element(distances.begin(), distances.end());
        dUpper = *max_element(distances.begin(), distances.end());
        std::cout << "Distance 2 Plane maximum value " <<dUpper<<std::endl;
        std::cout << "Distance 2 Plane minimum value " <<dLower<<std::endl;

        double cdLower, cdUpper;
        cdLower = *min_element(cen_distances.begin(), cen_distances.end());
        cdUpper = *max_element(cen_distances.begin(), cen_distances.end());
        std::cout << "Distance 2 Centroid maximum value " <<cdUpper<<std::endl;
        std::cout << "Distance 2 Centroid minimum value " <<cdLower<<std::endl;



        //the wieghts of the elements of h
        double alpha = 0.5;//0.1;//0.4;
        double beta = 0.1;//0.3;//0.4;
        double gama = 0.4;//0.6;//0.2;

        //evaluate the h function
        for (int i=0; i<cloud_cluster->points.size () ; i++)
        {
            double value = (alpha*(distances[i]/ dUpper)) + (beta*((pi-angles[i])/pi)) + (gama*(1-((cen_distances[i])/cdUpper)));
            // std::cout << "h angles value " <<(distances[i]/ dUpper)<<std::endl;
            //  std::cout << "h distance value " <<((pi-angles[i])/pi)<<std::endl;
            // std::cout << "h cen_distance value " <<(1-((cen_distances[i])/cdUpper))<<std::endl;
            h.push_back(value);
            // std::cout << "h value "<<i<<" " <<h[i]<<std::endl;

        }
        double upper_h, index;
        upper_h = *max_element(h.begin(), h.end());

        //TODO:: FIX ME
        for (int i=0; i<cloud_cluster->points.size () ; i++)
        {
            if(h[i]==upper_h)
            {index=i;break;}
        }
        //        vector<double>::iterator it ;
        //        it = find (h.begin(), h.end(), upper_h);
        //        const double N = sizeof(h) / sizeof(double);
        //        std::cout << "Index of max element another way: "
        //                  << std::distance(h.begin(),)
        //               << std::endl;
        std::cout << "Index of max element: "<<index<<std::endl;

        //visualization of the selected point
        //***************visulaization using the markers **************************************
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::ARROW;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        // normals orientation
        //        geometry_msgs::Quaternion msg;
        //        tf::Vector3 axis_vector(cloud_normals->points[index].normal_x, cloud_normals->points[index].normal_y, cloud_normals->points[index].normal_z);
        //        tf::Vector3 up_vector(0.0, 0.0, 1.0);
        //        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        //        right_vector.normalized();
        //        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        //        q.normalize();
        //        tf::quaternionTFToMsg(q, msg);




        // normals orientation
        Eigen::Vector3d axis_vector;
        geometry_msgs::Pose output_vector;
        Eigen::Quaterniond q;
        axis_vector[0] = cloud_normals->points[index].normal_x;
        axis_vector[1] = cloud_normals->points[index].normal_y;
        axis_vector[2] = cloud_normals->points[index].normal_z;
        axis_vector.normalize();

        //        Eigen::Vector3d up_vector(0.0, 0.0, -1.0);
        Eigen::Vector3d up_vector(0.0, 0.0, -1.0);//changes the orientation of the normal
        Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
        right_axis_vector.normalized();
        double theta = axis_vector.dot(up_vector);
        double angle_rotation = -1.0*acos(theta);
        tf::Vector3 tf_right_axis_vector;
        tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);
        tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);
        tf::quaternionTFToEigen(tf_q, q);
        Eigen::Affine3d pose;
        q.normalize();
        pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
        Eigen::Vector3d a;
        a[0]= cloud_cluster->points[index].x;//cloud_cluster->points[index].x;
        a[1]= cloud_cluster->points[index].y;//cloud_cluster->points[index].y;
        a[2]= cloud_cluster->points[index].z;//cloud_cluster->points[index].z;
        pose.translation() = a;
        tf::poseEigenToMsg(pose, output_vector);


        geometry_msgs::Pose fixedPose;
        //visulaization using the markers
        marker.scale.x = 0.2* (h[index]/3);
        marker.scale.y = 0.003;
        marker.scale.z = 0.003;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        fixedPose.position.x = cloud_cluster->points[index].x;
        fixedPose.position.y = cloud_cluster->points[index].y;
        fixedPose.position.z = cloud_cluster->points[index].z;



        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = index;
        ROS_INFO("Publishing Marker");
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.pose =  output_vector;//fixedPose;
        marker.pose.orientation  = output_vector.orientation;
        std::cout << " normal position x " <<output_vector.position.x<<std::endl;
        std::cout << " normal position y " <<output_vector.position.y<<std::endl;
        std::cout << " normal position z " <<output_vector.position.z<<std::endl;
        std::cout << " normal orientation x " <<output_vector.orientation.x<<std::endl;
        std::cout << " normal orientation y " <<output_vector.orientation.y<<std::endl;
        std::cout << " normal orientation z " <<output_vector.orientation.z<<std::endl;
        std::cout << " normal orientation w " <<output_vector.orientation.z<<std::endl;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.lifetime = ros::Duration(5);
        // Publish the marker
        marker_pub.publish(marker);

        //normal second point visualization ************************

        // point along the normal
        pcl::PointXYZ normal_point;

        normal_point.x = (cloud_normals->points[index].normal_x)/-15 + (cloud_cluster->points[index].x);
        normal_point.y = (cloud_normals->points[index].normal_y)/-15 + (cloud_cluster->points[index].y);
        normal_point.z = (cloud_normals->points[index].normal_z)/-15 + (cloud_cluster->points[index].z);

        visualization_msgs::Marker pointmarker;
        pointmarker.type = shape;
        pointmarker.action = visualization_msgs::Marker::ADD;

        pointmarker.scale.x = 0.2*(h[index]/3);
        pointmarker.scale.y = 0.01;
        pointmarker.scale.z = 0.01;
        // Set the color -- be sure to set alpha to something non-zero!
        pointmarker.color.r = 1.0f;
        pointmarker.color.g = 0.0f;
        pointmarker.color.b = 0.0f;
        pointmarker.color.a = 1.0;
        pointmarker.ns = "basic_shapes4";
        pointmarker.id = 5;
        fixedPose.position.x = normal_point.x;
        fixedPose.position.y = normal_point.y;
        fixedPose.position.z = normal_point.z;
        ROS_INFO("Publishing Marker point");
        std::cout << "projected Point along normal x " <<normal_point.x<<std::endl;
        std::cout << "projected Point along normal y " <<normal_point.y<<std::endl;
        std::cout << "projected Point along normal z " <<normal_point.z<<std::endl;
        pointmarker.pose =  fixedPose;
        pointmarker.pose.orientation =  output_vector.orientation;
        pointmarker.header.frame_id = "base_link";
        pointmarker.header.stamp = ros::Time::now();
        pointmarker.lifetime = ros::Duration();
//        marker_pub.publish(pointmarker);
        //**********************************************************



        //table orientation
        axis_vector[0] = plane_normal.normal_x;
        axis_vector[1] = plane_normal.normal_y;
        axis_vector[2] = plane_normal.normal_z;
        axis_vector.normalize();
        Eigen::Vector3d up_vector1(0.0, 0.0, 1.0);
        right_axis_vector = axis_vector.cross(up_vector1);
        right_axis_vector.normalized();
        theta = axis_vector.dot(up_vector1);
        angle_rotation = -1.0*acos(theta);
        tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);
        tf::Quaternion tf_q1(tf_right_axis_vector, angle_rotation);
        tf::quaternionTFToEigen(tf_q1, q);
        q.normalize();
        pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
        a[0]=0.1;
        a[1]=0;
        a[2]=0;
        pose.translation() = a;
        tf::poseEigenToMsg(pose, output_vector);


        visualization_msgs::Marker tablemarker;
        visualization_msgs::Marker centroidmarker;
        tablemarker.type = shape;
        tablemarker.action = visualization_msgs::Marker::ADD;
        centroidmarker.type = shape;
        centroidmarker.action = visualization_msgs::Marker::ADD;

        tablemarker.scale.x = 0.2;
        tablemarker.scale.y = 0.01;
        tablemarker.scale.z = 0.01;
        // Set the color -- be sure to set alpha to something non-zero!
        tablemarker.color.r = 1.0f;
        tablemarker.color.g = 0.0f;
        tablemarker.color.b = 0.0f;
        tablemarker.color.a = 1.0;

        fixedPose.position.x = 0.5;
        fixedPose.position.y = 0;
        fixedPose.position.z = 0;

        tablemarker.ns = "basic_shapes2";
        tablemarker.id = 1;
        ROS_INFO("Publishing Marker table");
        tablemarker.pose =  fixedPose;
        tablemarker.pose.orientation =  output_vector.orientation;
        tablemarker.header.frame_id = "base_link";
        tablemarker.header.stamp = ros::Time::now();
        tablemarker.lifetime = ros::Duration();
        //  marker_pub.publish(tablemarker);


        centroidmarker.scale.x = 0.05;
        centroidmarker.scale.y = 0.01;
        centroidmarker.scale.z = 0.01;
        // Set the color -- be sure to set alpha to something non-zero!
        centroidmarker.color.r = 0.0f;
        centroidmarker.color.g = 0.0f;
        centroidmarker.color.b = 1.0f;
        centroidmarker.color.a = 1.0;

        fixedPose.position.x = centroid[0];
        fixedPose.position.y = centroid[1];
        fixedPose.position.z = centroid[2];

        centroidmarker.ns = "basic_shapes3";
        centroidmarker.id = 2;
        ROS_INFO("Publishing Marker centroid");
        centroidmarker.pose =  fixedPose;
        centroidmarker.header.frame_id = "base_link";
        centroidmarker.header.stamp = ros::Time::now();
        centroidmarker.lifetime = ros::Duration();
        //  marker_pub.publish(centroidmarker);
        //********************************************************************************
        ros::Time end = ros::Time::now();
        double elapsed =  end.toSec() -begin.toSec();
        ROS_INFO("Time took for one:%f",elapsed);
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}
