#include "ProcessSensorData.hpp"

//STL
#include <cstddef>
#include <cassert>
#include <cstdio>

// DEBUG
#include <thread>
#include <chrono>

// (NEW) ROS2
#include <sensor_msgs/msg/camera_info.hpp>

//PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <thread>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

// OpenCV
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


// Helper to convert sensor_msgs::msg::PointCloud2 object to 
// pcl::PointCloud<pcl::PointXYZ>
inline void projectToPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_cloud) 
{
    cloud->clear();
    cloud->width = msg_cloud->width;
    cloud->height = msg_cloud->height;   

    cloud->is_dense = false;

    const uint8_t* data_ptr = msg_cloud->data.data();
    const uint32_t point_step = msg_cloud->point_step;

    for (size_t i = 0; i < msg_cloud->width * msg_cloud->height; ++i) {
        float x = 0.0f, y = 0.0f, z = 0.0f;

        // extract points and save in point cloud
        for (const auto& field : msg_cloud->fields) {
            if (field.name == "x") {
                memcpy(&x, data_ptr + field.offset, sizeof(float));
            } else if (field.name == "y") {
                memcpy(&y, data_ptr + field.offset, sizeof(float));
            } else if (field.name == "z") {
                memcpy(&z, data_ptr + field.offset, sizeof(float));
            }
        }

        // Put coords into pcl::PointCloud<pcl::PointXYZ>::Ptr
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);

        // Move ptr to next point in data
        data_ptr += point_step;
    }
}

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


ProcessSensorData::ProcessSensorData(const ParametersPtr &params) : 
    params_(params), outliers(0)
{
    if (params_->visualize)
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Cloud viewer"));
        viewer->initCameraParameters();
    }
}

void ProcessSensorData::segmentPlanes(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_cloud,
                                 PlanesList &planes)
{   
    //2. Get planes
    //2.1 Get point cloud
    //2.1.1 Compute 3D point cloud
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    projectToPointCloud(cloud, msg_cloud);

    //2.1.2 Filter point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::FastBilateralFilter<pcl::PointXYZ> bf;
    bf.setSigmaS(params_->sigma_s);
    bf.setSigmaR(params_->sigma_r);
    bf.setInputCloud(cloud);
    bf.filter(*filtered_cloud);  
    
    if(params_->verbose) 
    {
        if (filtered_cloud->empty()) {
            std::cerr << "Filtered cloud is empty." << std::endl;
        }
    }
//    int v1;
//    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer.setBackgroundColor(1.0, 1.0, 1.0, v1);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 0, 0, 0);
//    viewer.addPointCloud(cloud, color1, "cloud1", v1);

//    int v2;
//    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//    viewer.setBackgroundColor(0, 0, 0, v2);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(filtered_cloud, 0,255, 0);
//    viewer.addPointCloud(filtered_cloud, color2, "cloud2", v2);

//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
//    viewer.addCoordinateSystem(0.1);

    if (params_->visualize)
    {
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(filtered_cloud, 0, 0, 0);
        viewer->addPointCloud(filtered_cloud, cloud_color, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem(0.1);
    }

    //    while (!viewer.wasStopped())
    //    {
    //        viewer.spinOnce(100);
    //        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //    }
    //    viewer.close();

    //2.2. Segment planes
    //2.2.1 Compute normals
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;          
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    
    //ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
    //ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    //ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(params_->max_change_factor);      // max relative depth change to consider two points as part of same surface
    ne.setNormalSmoothingSize(params_->smoothing_size);          // neighbourhood of a point that is used to compute its noormal
    ne.setDepthDependentSmoothing(true);                                        

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(filtered_cloud);  

    ne.compute(*normal_cloud);                          // compute normmal vector for every point in cloud                                              

//    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (filtered_cloud, normal_cloud, 100, 0.02, "normals2", v2);
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals2");

    //2.2.2 Plane segmentation
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(filtered_cloud->size()*params_->inliers_ratio);        // min number of points to form a plane
    mps.setAngularThreshold(params_->angular_th);                            // angular diff of normal vectors to assign two pts to same plane
    mps.setDistanceThreshold(params_->distance_th);                          // max perpendicular distance to consider a point as part of a plane
    mps.setInputNormals(normal_cloud);                              
    mps.setInputCloud(filtered_cloud);

    //Using segmentAndRefine produces a plane estimation with more error (undesired), but with more inliers (desired)
    std::vector< pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator< pcl::PlanarRegion<pcl::PointXYZ> > > regions;
    // mps.segment(regions);
    mps.segmentAndRefine(regions);

    //    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    //    std::vector<pcl::ModelCoefficients> model_coefficients;
    //    std::vector<pcl::PointIndices> inliers;
    //    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    //    std::vector<pcl::PointIndices> label_indices;
    //    std::vector<pcl::PointIndices> boundary_indices;
    //    mps.segmentAndRefine(regions, model_coefficients, inliers, labels, label_indices, boundary_indices);

    planes.clear();
    planes.resize(regions.size());

    // 2.2.3 Compute plane parameters
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

        //Note: the following step is unnessesary, but done here just to make sure all plane normals have the same sign.
        if (normal.dot(centroid) > 0)
          normal = -normal;

        planes[k].n = normal/normal.norm();
        planes[k].d = -planes[k].n.dot(centroid);

//        std::cout << "N: " << planes[k].n.transpose() << std::endl;
//        std::cout << "D: " << planes[k].d << std::endl;

        if (params_->visualize)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            boundary_cloud->points = regions[k].getContour();

//            Warning: Showing only the last plane!!
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(boundary_cloud, 0, 255, 255);
//            viewer.addPointCloud(boundary_cloud, color, "plane", v2);
//            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundary_color(boundary_cloud, 0, 255, 255);
            viewer->addPointCloud(boundary_cloud, boundary_color, "plane");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane");

//            TODO: show plane normal

            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
            viewer->resetStoppedFlag();

            viewer->removePointCloud("plane");
        }
     }

    if (params_->visualize) viewer->removePointCloud("cloud");

}

void ProcessSensorData::segmentPlanes(sensor_msgs::msg::Image::ConstSharedPtr msg_image,
                                      PlanesList &planes)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);

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
    // cv::Mat D = (cv::Mat_<double>(1, 5) << params_->D[0],
    //     params_->D[1], params_->D[2], params_->D[3], params_->D[4]);
    cv::Mat D(params_->D, true);

    bool found = true;

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // Iterate to detect checkerboards and draw over them after detection
    while (found) 
    {
        std::vector<cv::Point> corners;
        found = cv::findChessboardCorners(gray_image, cv::Size(params_->checkerboardWidth, params_->checkerboardHeight), corners);

        if (found) 
        {
            // Convert to float points for refinement
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

            // Compute normal vector (n) and distance (d) of the checkerboard plane in camera coordinate system
            cv::Mat R;
            cv::Rodrigues(rvec, R); // Convert rotation vector to rotation matrix

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
            // std::cout << "Normal vector of checkerboard: "<< n[0] << ", " << n[1] << ", " << n[2] << std::endl;
            // std::cout << "dist to orig. of checkerboard: "<< d << std::endl;

            // Add to planes list
            Plane new_plane;
            new_plane.n = n;
            new_plane.d = d;
            planes.push_back(new_plane);

            // Remove checkerboard from image by painting over it
            std::vector< std::vector<cv::Point> > hull(1);
            cv::convexHull(corners, hull[0]);
            cv::fillConvexPoly(gray_image, hull[0], 255);
        }
    }
}

Correspondences ProcessSensorData::findFromPlanesList(const ObservationPair &obs2, std::string label1, std::string label2, const PlanesList &p1, const PlanesList &p2, bool use_initial_est)
{
    // Extract frame IDs
    std::size_t id_i, id_j;
    id_i = params_->sensor_id[label1];
    id_j = params_->sensor_id[label2];
    // std::visit([&id_i, this](const auto& obs) {
    //     id_i = params_->sensor_id[obs->header.frame_id];
    //     std::cout << "id_i frame id = " << obs->header.frame_id << std::endl;
    // }, obs2.first);
    // std::visit([&id_j, this](const auto& obs) {
    //     id_j = params_->sensor_id[obs->header.frame_id];
    //     std::cout << "id_j frame id = " << obs->header.frame_id << std::endl;
    // }, obs2.second);

    assert(id_i != id_j);

    PlanesList p_i = p1;
    PlanesList p_j = p2;
    if (id_i > id_j)
    {
        std::swap(id_i, id_j);
        std::swap(p_i, p_j);
    }

    Correspondences matches;
    for (std::size_t i = 0; i < p_i.size(); ++i)
    {
        Eigen::Vector3f n_i = undoHomogeneousVector(params_->init_pose[id_i]*getHomogeneousVector(p_i[i].n));   // Transform normal vector
        Eigen::Vector3f x_i = -p_i[i].n*p_i[i].d;                                                               // Compute a point on the plane (assuming norm(n) == 1)
        float d_i = -n_i.dot(undoHomogeneousPoint(params_->init_pose[id_i]*getHomogeneousPoint(x_i)));          // Calculate distance in new coord system

        //  Search other PlanesList for corresponding plane
        for (std::size_t j = 0; j < p_j.size(); ++j)
        {
            Eigen::Vector3f n_j, x_j;
            float d_j;
            std::visit([&n_j, &x_j, &d_j, &p_j, &id_j, &j, this](const auto&) {
                n_j = undoHomogeneousVector(params_->init_pose[id_j]*getHomogeneousVector(p_j[j].n));
                x_j = -p_j[j].n*p_j[j].d;
                d_j = -n_j.dot(undoHomogeneousPoint(params_->init_pose[id_j]*getHomogeneousPoint(x_j)));
            }, obs2.second);

            float a = std::abs(std::acos(n_i.dot(n_j)) * 180.0 / M_PI);     // Compare planes by normal vector difference
            float d = std::abs(d_i-d_j);                                    // Compare planes by distance to origin

            //TODO: the propagation of the estimated error is not really rigorous...
            if ((a < params_->max_angle) && (d < params_->max_distance)) {
                matches.push_back(PlaneMatch(p_i[i], p_j[j]));
                if (params_->verbose) 
                {
                    std::cout << "INLIER FOUND for (" << id_i << ", " << id_j << ")." << std::endl;
                    // std::cout << "first: n = (" << n_i[0] << ", " << n_i[1] << ", " << n_i[2] << "), d = " << d_i << std::endl;
                    // std::cout << "second: n = (" << n_j[0] << ", " << n_j[1] << ", " << n_j[2] << "), d = " << d_j << std::endl;
                    // std::cout << "Angle diff: " << a << ", dist diff: " << d << std::endl;
                }
            }

            else if (!use_initial_est)
            {
                outliers++;
                // matches.push_back(PlaneMatch(p_i[i], p_j[j]));
                if (params_->verbose) 
                {
                    std::cout << "OUTLIER DETECTED for (" << id_i << ", " << id_j << ")." << std::endl;
                    std::cout << "Angle diff: " << a << ", dist diff: " << d << std::endl << std::endl;
                }
            }
            // else 
            // {
            //     std::cout << "Planes don't match. Error in normal vector: " << a << ", error in distance: " << d << "." << std::endl;
            // }
        }
    }
    // std::cout << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return matches;
}
