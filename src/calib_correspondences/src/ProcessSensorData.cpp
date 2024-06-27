#include "ProcessSensorData.hpp"

//STL
#include <cstddef>
#include <cassert>
#include <cstdio>

// DEBUG
#include <thread>
#include <chrono>

//PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/fast_bilateral.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <omp.h>

// More helper functions
inline Eigen::Vector4f getHomogeneousPoint(const Eigen::Vector3f &v)
{
    return Eigen::Vector4f(v(0), v(1), v(2), 1.f);
}

inline Eigen::Vector3f undoHomogeneousPoint(const Eigen::Vector4f &h)
{
    return Eigen::Vector3f(h(0)/h(3), h(1)/h(3), h(2)/h(3));
}

inline Eigen::Vector4f getHomogeneousVector(const Eigen::Vector3f &v)
{
    return Eigen::Vector4f(v(0), v(1), v(2), 0.f);
}

inline Eigen::Vector3f undoHomogeneousVector(const Eigen::Vector4f &h)
{
    Eigen::Vector3f v(h(0), h(1), h(2));
    return v/v.norm();
}

// -------------------------------------------------------------------------- //


ProcessSensorData::ProcessSensorData(const ParametersPtr &params, rclcpp::Node& node) : 
    params_(params), outliers(0), node_(node)
{
    // Visualization
    std::size_t img_counter = params_->getNumImgSensors();
    vis_planes_img_.resize(img_counter);
    std::size_t pc2_counter = params_->getNumCloudSensors();
    vis_planes_pc_.resize(pc2_counter);
    vis_marker_.resize(1);

    for (std::size_t i = 0; i < img_counter; ++i)
        vis_planes_img_[i] = node_.create_publisher<sensor_msgs::msg::Image>("/visualize/segmented_image" + std::to_string(i), 10);
    for (std::size_t j = 0; j < pc2_counter; ++j)
        vis_planes_pc_[j] = node.create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/segmented_cloud" + std::to_string(j), 10);
    for (std::size_t k = 0; k < vis_marker_.size(); ++k)
        vis_marker_[k] = node.create_publisher<visualization_msgs::msg::Marker>("/visualize/cloud_marker" + std::to_string(k), 10);

    marker_id_counter.resize(1); // We need one counter for every point cloud (?)
    for(std::size_t idx = 0; idx < marker_id_counter.size(); ++idx)
        marker_id_counter[idx] = 100000 *(idx + 1);
        
}

void ProcessSensorData::segmentPlanes(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_cloud,
                                 PlanesList &planes, std::size_t id)
{   
    //2. Get planes
    //2.1 Get point cloud
    //2.1.1 Compute 3D point cloud
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // auto start1 = std::chrono::high_resolution_clock::now();
    pcl::fromROSMsg(*msg_cloud, *cloud);
    // auto end2 = std::chrono::high_resolution_clock::now();
    // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start1);
    // std::cout << "fromROSMsg(): " << duration2.count() << " ms " << std::endl;
            

    //2.1.2 Filter point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::FastBilateralFilter<pcl::PointXYZ> bf;
    bf.setSigmaS(params_->sigma_s);
    bf.setSigmaR(params_->sigma_r);
    bf.setInputCloud(cloud);
    
    // start1 = std::chrono::high_resolution_clock::now();
    bf.filter(*filtered_cloud); 
    // end2 = std::chrono::high_resolution_clock::now();
    // duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start1);
    // std::cout << "bf.filter(): " << duration2.count() << " ms " << std::endl;
    
    if(params_->verbose) 
    {
        if (filtered_cloud->empty()) {
            std::cerr << "Filtered cloud is empty." << std::endl;
        }
    }

    //2.2. Segment planes
    //2.2.1 Compute normals

    // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;          
    // ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    // ne.setMaxDepthChangeFactor(params_->max_change_factor);      // max relative depth change to consider two points as part of same surface
    // ne.setNormalSmoothingSize(params_->smoothing_size);          // neighbourhood of a point that is used to compute its noormal
    // ne.setDepthDependentSmoothing(true);                                        
    // pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    // ne.setInputCloud(filtered_cloud);
    // ne.compute(*normal_cloud);                          // compute normmal vector for every point in cloud
    
    // Normals using NormalEstimation
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setKSearch(150);
    ne.setNumberOfThreads(omp_get_max_threads());
    // ne.setRadiusSearch(params_->smoothing_size);
    ne.setInputCloud(filtered_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    // start1 = std::chrono::high_resolution_clock::now();
    ne.compute(*normal_cloud);
    // end2 = std::chrono::high_resolution_clock::now();
    // duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start1);
    // std::cout << "ne.compute(): " << duration2.count() << " ms" << std::endl;


    //2.2.2 Plane segmentation
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(filtered_cloud->size()*params_->inliers_ratio);        // min number of points to form a plane
    mps.setAngularThreshold(params_->angular_th);                            // angular diff of normal vectors to assign two pts to same plane
    mps.setDistanceThreshold(params_->distance_th);                          // max perpendicular distance to consider a point as part of a plane
    mps.setInputNormals(normal_cloud);                            
    mps.setInputCloud(filtered_cloud);

    //Using segmentAndRefine produces a plane estimation with more error (undesired), but with more inliers (desired)
    std::vector< pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator< pcl::PlanarRegion<pcl::PointXYZ> > > regions;

    // Initialize the vectors and shared pointer
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    auto labels = std::make_shared<pcl::PointCloud<pcl::Label>>();
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    // start1 = std::chrono::high_resolution_clock::now();
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    // end2 = std::chrono::high_resolution_clock::now();
    // duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start1);
    // std::cout << "mps.segmentAndRefine(): " << duration2.count() << " ms " << std::endl;

    

    planes.clear();
    planes.resize(regions.size());

    // 2.2.3 Compute plane parameters
    // std::cout << regions.size() << " regions detected." << std::endl;
    for (std::size_t k = 0; k < regions.size(); ++k)
    {
        Eigen::Vector4f model = regions[k].getCoefficients();
        //model coefficients:
        //  model[0]: normal.x
        //  model[1]: normal.y
        //  model[2]: normal.z
        //  model[3]: d

        // if (verbose)
        //     std::cout << "model: " << model.transpose() << std::endl;

        Eigen::Vector3f normal(model[0], model[1], model[2]);
        Eigen::Vector3f centroid = regions[k].getCentroid();

        //Note: the following step is unnessesary, but done here just to make sure all plane normals have the same sign (point towards origin)
        if (normal.dot(centroid) > 0)
          normal = -normal;

        planes[k].n = normal/normal.norm();
        planes[k].d = -planes[k].n.dot(centroid);
        planes[k].c = centroid;
        // std::cout << "PLANE IN CLOUD DETECTED." << std::endl;
    }

    // Visualization
    if (params_->visualize)
    {
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        colored_cloud.resize(filtered_cloud->size());
        
        // Set all points to white initially
        for (size_t i = 0; i < filtered_cloud->size(); ++i)
        {
            colored_cloud.points[i].x = filtered_cloud->points[i].x;
            colored_cloud.points[i].y = filtered_cloud->points[i].y;
            colored_cloud.points[i].z = filtered_cloud->points[i].z;
            colored_cloud.points[i].r = 255;
            colored_cloud.points[i].g = 255;
            colored_cloud.points[i].b = 255;
        }

        
        // Clear all makers in the RViz2 view
        // deleteAllMarkers(id);

        // Color the points in each plane with a unique color
        for (size_t i = 0; i < inlier_indices.size(); ++i)
        {
            auto r = rand() % 256;
            auto g = rand() % 256;
            auto b = rand() % 256;

            for (auto idx : inlier_indices[i].indices)
            {
                // Color each plane
                colored_cloud.points[idx].r = r;
                colored_cloud.points[idx].g = g; 
                colored_cloud.points[idx].b = b;
            }

        }
        
        sensor_msgs::msg::PointCloud2 pc2_colored;
        pcl::toROSMsg(colored_cloud, pc2_colored);
        pc2_colored.header.frame_id = "map";

        // As PointCloud2 sensors are listed first this should work
        vis_planes_pc_[id]->publish(pc2_colored);

        // // Visualize normal vector
        // for (auto &plane : planes)
        //     publishPlaneMarker(plane.n, plane.c, id);
    }
}

void ProcessSensorData::segmentPlanes(sensor_msgs::msg::Image::ConstSharedPtr msg_image,
                                      PlanesList &planes, std::size_t id)
{
    cv_bridge::CvImagePtr cv_ptr, image_visualize;

    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);    
    image_visualize = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(image_visualize->image, image_visualize->image, cv::COLOR_BGR2GRAY);
    
    double C = 4.0;
    int window = image_visualize->image.cols * 0.1;
    window = window % 2 == 1 ? window : window - 1;     // make sure it's not even
    cv::adaptiveThreshold(image_visualize->image, image_visualize->image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, window, C);
    cv::cvtColor(image_visualize->image, image_visualize->image, cv::COLOR_GRAY2BGR);

    if (cv_ptr->image.empty())
    {
        std::cerr << "Empty image received." << std::endl;
        return;
    }

    cv::Mat image = cv_ptr->image;

    // Get intrinsic camera parameters
    cv::Mat K = (cv::Mat_<double>(3, 3) <<  
                    params_->fx,    0,              params_->cx, 
                    0,              params_->fy,    params_->cy, 
                    0,              0,              1);
    cv::Mat D(params_->D, true);

    bool found = true;

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat binarized_image;
    cv::adaptiveThreshold(gray_image, binarized_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, window, C);
    gray_image = binarized_image;

    // Iterate to detect checkerboards and draw over them after detection
    while (found) 
    {
        std::vector<cv::Point> corners;
        found = cv::findChessboardCorners(gray_image, cv::Size(params_->checkerboardWidth, params_->checkerboardHeight), corners, cv::CALIB_CB_FAST_CHECK);

        if (found) 
        {
            // Convert to float points for refinement
            std::cout << "Chessboard detected." << std::endl;
            std::vector<cv::Point2f> refined_corners(corners.begin(), corners.end());

            // Refine corner locations
            cv::cornerSubPix(gray_image, refined_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            // Define object points (3D points of checkerboard corners in checkerboard coordinate system)
            std::vector<cv::Point3f> objectPoints;
            for (int i = 0; i < params_->checkerboardHeight; ++i) {
                for (int j = 0; j < params_->checkerboardWidth; ++j) {
                    objectPoints.push_back(cv::Point3f(j * params_->squareSize, i * params_->squareSize, 0));
                }
            }

            // Estimate pose of checkerboard
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, refined_corners, K, D, rvec, tvec);

            // Compute normal vector and distance of the checkerboard plane in camera coordinate system
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            // Extract third column of rotation matrix (z-axis)
            cv::Mat normalVector = R.col(2);

            // Translation vector
            cv::Mat translationVector = tvec;
            if (normalVector.dot(translationVector) > 0)
                normalVector = -normalVector;

            // The plane equation is ax + by + cz + d = 0, where (a, b, c) is the normal vector (n)
            // and d is the distance to the origin
            double d = -normalVector.dot(translationVector); // Distance (d)

            Eigen::Vector3f n(normalVector.at<double>(0), normalVector.at<double>(1), normalVector.at<double>(2));

            // Calculate centroid (for visualization)
            Eigen::Vector3f centroid(0., 0., 0.);
            for (const auto& point : objectPoints) 
            {
                cv::Mat pointMat = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
                cv::Mat transformedPoint = R * pointMat + tvec;
                centroid[0] += transformedPoint.at<double>(0);
                centroid[1] += transformedPoint.at<double>(1);
                centroid[2] += transformedPoint.at<double>(2);
            }
            centroid /= objectPoints.size();
            
            cv::Scalar color(rand()%256, rand()%256, rand()%256);
            if (params_->visualize)
            {
                // // Calculate endpoint of the normal vector in 3D space (for visualization)
                // float normalLength = 0.75 * params_->checkerboardWidth * params_->squareSize;
                // Eigen::Vector3f endpoint = centroid + normalLength * n;

                // cv::Mat_<double> endpointMat = (cv::Mat_<double>(3, 1) << endpoint[0], endpoint[1], endpoint[2]);
                // cv::Mat projectedPointMat = K * endpointMat; // Apply camera intrinsic matrix
                // cv::Point2f projectedPoint(projectedPointMat.at<double>(0) / projectedPointMat.at<double>(2),
                //                         projectedPointMat.at<double>(1) / projectedPointMat.at<double>(2));

                // // Draw the normal vector on the image
                // cv::line(image_visualize->image, cv::Point(centroid[0], centroid[1]), projectedPoint, color, 10);

                cv::Mat centroidMat = K * (cv::Mat_<double>(3, 1) << centroid[0], centroid[1], centroid[2]);
                cv::Point2f centroid_image;
                centroid_image.x = static_cast<float>(centroidMat.at<double>(0) / centroidMat.at<double>(2));
                centroid_image.y = static_cast<float>(centroidMat.at<double>(1) / centroidMat.at<double>(2));
                cv::circle(image_visualize->image, centroid_image, 20, color, -1);

                // Calculate endpoint of the normal vector in 3D space (for visualization)
                float normalLength = 0.75 * params_->checkerboardWidth * params_->squareSize;
                Eigen::Vector3f endpoint = centroid + normalLength * n;
                cv::Mat endpointMat = K * (cv::Mat_<double>(3, 1) << endpoint[0], endpoint[1], endpoint[2]);

                cv::Point2f endpoint_image(endpointMat.at<double>(0) / endpointMat.at<double>(2),
                                           endpointMat.at<double>(1) / endpointMat.at<double>(2));

                cv::arrowedLine(image_visualize->image, centroid_image, endpoint_image, color, 10, 8, 0, 0.2 * normalLength);
            }

            // Add to planes list
            Plane new_plane;
            new_plane.n = n;
            new_plane.d = d;
            new_plane.c = centroid;
            planes.push_back(new_plane);

            // Remove checkerboard from image by painting over it
            std::vector< std::vector<cv::Point> > hull(1);
            cv::convexHull(corners, hull[0]);
            cv::fillConvexPoly(gray_image, hull[0], 255);
            if(params_->visualize)
            {
                // cv::Scalar color(rand()%256, rand()%256, rand()%256);
                cv::polylines(image_visualize->image, hull[0], true, color, 12);
            }
        }
    }

    if (params_->visualize)
    {
        sensor_msgs::msg::Image::SharedPtr ros_image_planes = image_visualize->toImageMsg();
        // As PointCloud2 sensors are listed first this should work. 
        // The index of the image publisher should be its general sensor id minus the number of cloud sensors
        vis_planes_img_[id - params_->getNumCloudSensors()]->publish(*ros_image_planes);
    }


}

Correspondences ProcessSensorData::findFromPlanesList(const ObservationPair &obs2, 
                                                      std::string topic1, std::string topic2, 
                                                      const PlanesList &p1, const PlanesList &p2,
                                                      bool use_initial_est)
{
    // Extract frame IDs
    std::size_t id_i, id_j;
    id_i = params_->sensor_id[topic1];
    id_j = params_->sensor_id[topic2];

    assert(id_i != id_j);

    PlanesList p_i = p1;
    PlanesList p_j = p2;
    if (id_i > id_j)
    {
        std::swap(id_i, id_j);
        std::swap(p_i, p_j);
    }

    // Clean RViz2 (visualization)
    if (params_->msg_types[id_i] == "PointCloud2")
        deleteAllMarkers(id_i);
    if (params_->msg_types[id_j] == "PointCloud2")
        deleteAllMarkers(id_j);

    Correspondences matches;
    for (std::size_t i = 0; i < p_i.size(); ++i)
    {
        Eigen::Matrix4f T_i = params_->init_pose[id_i];
        Eigen::Vector3f n_i = T_i.block<3,3>(0,0) * p_i[i].n;
        Eigen::Vector3f x_i = -p_i[i].n * p_i[i].d;                                          // Compute a point on the plane (assuming norm(n) == 1)
        float d_i = -n_i.dot(undoHomogeneousPoint(T_i * getHomogeneousPoint(x_i)));          // Calculate distance in new coord system

        //  Search other PlanesList for corresponding plane
        for (std::size_t j = 0; j < p_j.size(); ++j)
        {
            Eigen::Matrix4f T_j = params_->init_pose[id_j];
            Eigen::Vector3f n_j, x_j;
            float d_j;
            std::visit([&T_j, &n_j, &x_j, &d_j, &p_j, &id_j, &j, this](const auto&) {
                n_j = T_j.block<3,3>(0,0) * p_j[j].n;
                x_j = -p_j[j].n * p_j[j].d;
                d_j = -n_j.dot(undoHomogeneousPoint(T_j * getHomogeneousPoint(x_j)));
            }, obs2.second);

            float a = std::abs(std::acos(n_i.dot(n_j)) * 180.0 / M_PI);     // Compare planes by normal vector difference
            float d = std::abs(d_i-d_j);                                    // Compare planes by distance to origin

            //TODO: the propagation of the estimated error is not really rigorous...
            if ((a < params_->max_angle) && (d < params_->max_distance)) {
                matches.push_back(PlaneMatch(p_i[i], p_j[j]));

                if (params_->verbose) 
                    std::cout << "INLIER FOUND for (" << id_i << ", " << id_j << ")." << std::endl;

                if (params_->visualize)
                {       
                        Eigen::Vector3f c_i = undoHomogeneousPoint(T_i * getHomogeneousPoint(p_i[i].c));
                        Eigen::Vector3f c_j = undoHomogeneousPoint(T_j * getHomogeneousPoint(p_j[j].c));


                    if (params_->msg_types[id_i] == "PointCloud2")
                        publishPlaneMarker(n_i, c_i, 0); 
                    else
                        publishPlaneMarker(n_i, c_i, 0, 255, 0, 0);
                    if (params_->msg_types[id_j] == "PointCloud2")
                        publishPlaneMarker(n_j, c_j, 0);
                    else   
                        publishPlaneMarker(n_j, c_j, 0, 0, 255, 0);

                }
            }
            else // planes do not match
            {
                std::cout << "NO MATCH." << std::endl;
                std::cout << "Angle diff:\t " << a << std::endl;
                std::cout << "Distance diff:\t " << d << std::endl;
            }
        }
    }
    return matches;
}


void ProcessSensorData::publishPlaneMarker(Eigen::Vector3f n, Eigen::Vector3f offset, std::size_t id, int r, int g, int b)
{
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.stamp = node_.now();
    marker.id = marker_id_counter[id]++;
    marker.ns = "found_plane_markers";

    marker.pose.position.x = offset[0];
    marker.pose.position.y = offset[1];
    marker.pose.position.z = offset[2];

    marker.scale.x = 0.7;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    marker.color.r = r / 255.0;
    marker.color.g = g / 255.0;
    marker.color.b = b / 255.0;
    marker.color.a = 1.0;
    
    Eigen::Vector3f direction = n;
    direction.normalize();

    Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), direction);
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w(); 

    vis_marker_[id]->publish(marker);
}


void ProcessSensorData::deleteAllMarkers(std::size_t id) 
{
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.header.stamp = node_.now();
    marker.id = 0;
    marker.ns = "found_plane_markers";
    vis_marker_[id]->publish(marker);
}

