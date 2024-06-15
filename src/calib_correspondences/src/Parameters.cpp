#include <fstream>
#include <yaml-cpp/yaml.h>

#include "Parameters.hpp"

Parameters::Parameters() {
    verbose = false;

    max_time_diff = 0.0;

    visualize = false;
    sigma_s = 0.0;
    sigma_r = 0.0;
    max_change_factor = 0.0;
    smoothing_size = 0.0;
    inliers_ratio = 0.0;
    angular_th = 0.0;
    distance_th = 0.0;

    fx = 0.0;                      
    fy = 0.0;                      
    cx = 0.0;                       
    cy = 0.0;                     
    checkerboardWidth = 0;          
    checkerboardHeight = 0;         
    squareSize = 0.0;              

    max_angle = 0.0;
    max_distance = 0.0;
    calib_strategy = "";
    algorithm = "";
    error_analysis = false;
    lim_correspondences = 1000;
}


void Parameters::addSensorPose(std::string topic, std::string msg_type,
                               std::vector<double> r, std::vector<double> t) {
    // Convert rotation and translation vectors to Eigen vectors
    Eigen::Vector3d tE(t.data());
    Eigen::Vector3d rE(r.data());

    // Create 4x4 SE(3) pose matrix
    Eigen::Matrix4f pose_i = create_se3(tE.cast<float>(), rE.cast<float>());

    // Add pose and topic to internal storage
    init_pose.push_back(pose_i);
    sensor_topics.push_back(topic);
    msg_types.push_back(msg_type);
}


void Parameters::loadFromYAML(const std::string& filename) {
    // Check if the file exists and can be opened
    std::ifstream fin(filename);
    if (fin.fail()) {
        if (fin.bad()) {
            // File cannot be opened (possibly due to permissions)
            std::cerr << "Error: YAML file '" << filename << "' cannot be opened." << std::endl;
        } else {
            // File not found
            std::cerr << "Error: YAML file '" << filename << "' not found." << std::endl;
        }
    }

    YAML::Node config = YAML::Load(fin);

    // Fetch parameters
    verbose = config["verbose"].as<bool>();
    visualize = config["visualize"].as<bool>();
    max_time_diff = config["max_time_diff"].as<double>();
    sigma_s = config["sigma_s"].as<double>();
    sigma_r = config["sigma_r"].as<double>();
    max_change_factor = config["max_change_factor"].as<double>();
    smoothing_size = config["smoothing_size"].as<double>();
    inliers_ratio = config["inliers_ratio"].as<double>();
    angular_th = config["angular_th"].as<double>();
    distance_th = config["distance_th"].as<double>();

    fx = config["fx"].as<double>();
    fy = config["fy"].as<double>();
    cx = config["cx"].as<double>();
    cy = config["cy"].as<double>();
    std::vector<double> D = config["D"].as<std::vector<double>>();
    checkerboardWidth = config["checkerboardWidth"].as<int>();
    checkerboardHeight = config["checkerboardHeight"].as<int>();
    squareSize = config["squareSize"].as<double>();

    max_angle = config["max_angle"].as<double>();
    max_distance = config["max_distance"].as<double>();
    calib_strategy = config["calib_strategy"].as<std::string>();
    algorithm = config["algorithm"].as<std::string>();      
    error_analysis = config["error_analysis"].as<bool>();
    lim_correspondences = config["lim_correspondences"].as<std::size_t>();

    // Process sensor data
    int num_sensors = config["num_sensors"].as<int>();

    for (int i = 0; i < num_sensors; ++i) 
    {
        std::string t_param_name = "t" + std::to_string(i);
        std::string r_param_name = "r" + std::to_string(i);
        std::string topic_param_name = "topic_" + std::to_string(i);
        std::string msgtype_param_name = "msgtype_" + std::to_string(i);

        std::vector<double> r = config[r_param_name].as<std::vector<double>>();
        std::vector<double> t = config[t_param_name].as<std::vector<double>>();
        std::string topic = config[topic_param_name].as<std::string>();
        std::string msg_type = config[msgtype_param_name].as<std::string>();

        // Add sensor pose to initialization parameters
        addSensorPose(topic, msg_type, r, t);
    }

    // Generate map after saving all sensor poses in param_
    genSensorIdMap();
}


void Parameters::genSensorIdMap() {
    std::size_t n = sensor_topics.size();
    assert(n > 1);
    for (std::size_t k = 0; k < n; ++k) {
        sensor_id[sensor_topics[k]] = k;
    }
}


std::size_t Parameters::getPairId(std::size_t id_i, std::size_t id_j)
{
    if (id_i > id_j)
        std::swap(id_i, id_j);

    assert(id_i < id_j);
    return (id_i*sensor_topics.size() + id_j - (id_i + 1)*(id_i + 2)/2);
}


void Parameters::getIndicesFromPairId(std::size_t vector_index, std::size_t& id_i, std::size_t& id_j) 
{
    id_i = 0;
    while (vector_index >= sensor_topics.size() - 1 - id_i) {
        vector_index -= sensor_topics.size() - 1 - id_i;
        id_i++;
    }
    id_j = id_i + 1 + vector_index;
}