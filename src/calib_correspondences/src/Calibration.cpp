#include "Calibration.hpp"

#include <Eigen/Geometry>

//STL
#include <cassert>
#include <cmath>
#include <algorithm>
#include <random>

//Debug only
#include <iostream>
#include <fstream>

#include "auxiliar.h"

/// Helper functions
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

Correspondences getRandomElements(std::size_t size, const Correspondences& corr) {
    // Check if l is greater than the size of the input vector
    if (size > corr.size()) {
        // std::cout << "WARNING: Not enough elements. " << corr.size()  << " were given, " << size << " were selected as lim_correspondences parameter." << std::endl;
        return corr;
    }

    // // If size is less than or equal to 10, ensure there are at least 3 elements between selected elements
    // if (size <= 10 && corr.size() > size) {
    //     // std::cout << "WARNING: Size is less than or equal to 10. Ensuring at least 3 elements between selected elements." << std::endl;
    //     std::size_t min_gap = 3; // Minimum gap between selected elements
    //     std::size_t gap = (corr.size() - min_gap * (size - 1)) / size;
    //     std::vector<std::size_t> indices;

    //     // Random number generator
    //     std::random_device rd;
    //     std::mt19937 gen(rd());

    //     // Generate random indices ensuring minimum gap
    //     std::uniform_int_distribution<std::size_t> dist(0, gap - 1);
    //     std::size_t start_index = dist(gen);
    //     for (std::size_t i = 0; i < size; ++i) {
    //         indices.push_back(start_index + i * (gap + min_gap));
    //         std::cout << start_index + i * (gap + min_gap) << std::endl;
    //     }

    //     // Copy selected elements into u
    //     Correspondences u;
    //     for (std::size_t index : indices) {
    //         std::cout << "push back vector." << std::endl;
    //         u.push_back(corr[index]);
    //     }

    //     std::cout << "Selected " << size << " elements for calibration randomly with at least 3 elements between them." << std::endl;
    //     return u;
    // }

    // If size is greater than 10 or there are not enough elements, proceed normally
    Correspondences u;

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Sample l elements from v and store them in u
    std::sample(corr.begin(), corr.end(), std::back_inserter(u), size, gen);
    std::cout << "Selected " << size << " elements for calibration randomly." << std::endl;
    return u;
}

float calculateMedian(const std::vector<float>& vec) {
    std::vector<float> sortedVec = vec; // Make a copy to avoid modifying the original vector
    std::sort(sortedVec.begin(), sortedVec.end());

    size_t size = sortedVec.size();
    if (size == 0) {
        std::cerr << "Error: Vector is empty." << std::endl;
        return 0; // Return 0 or handle the error appropriately
    }

    size_t middle = size / 2;
    if (size % 2 == 0) {
        // Even number of elements, average the two middle elements
        return (sortedVec[middle - 1] + sortedVec[middle]) / 2.0f;
    } else {
        // Odd number of elements, return the middle element
        return sortedVec[middle];
    }
}

std::vector<float> extractEulerAngles(const Eigen::Matrix3f& R) {
    std::vector<float> angles(3);
    
    // Extract pitch (beta)
    float pitch = std::asin(-R(2, 0));
    angles[1] = pitch * (180.0 / M_PI);  // pitch in degrees

    // Check for gimbal lock
    if (std::cos(pitch) != 0) {
        float roll = std::atan2(R(2, 1), R(2, 2));
        float yaw = std::atan2(R(1, 0), R(0, 0));
        
        angles[0] = roll * (180.0 / M_PI);  // roll in degrees
        angles[2] = yaw * (180.0 / M_PI);   // yaw in degrees
    } else {
        float roll = std::atan2(-R(1, 2), R(1, 1));
        angles[0] = roll * (180.0 / M_PI);  // roll in degrees
        angles[2] = 0;                      // yaw is not defined in this case
    }
    
    return angles;
}

// End of helper functions
// ---------------------------------------------------------------------------//


/// Constructor grabbing all data from params
Calibration::Calibration(const ParametersPtr &params, CorrespondenceVec& corr_vec)
    : all_matches(corr_vec), params_(params)
{
    Tini = Matrix4d::Identity();
}


//TODO: if id_i > id_j -> return inverse rotation
Eigen::Matrix3f Calibration::calibrateRotation(std::size_t id_i, std::size_t id_j)
{
    // Calibration system
    Eigen::Matrix3f rotationCov = Eigen::Matrix3f::Zero();
    float accum_error = 0.f;

    std::size_t idx = params_->getPairId(id_i, id_j);
    Correspondences matches = getRandomElements(params_->lim_correspondences, all_matches[idx]);
    PlaneMatch match;

    std::size_t N = matches.size();

    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        Eigen::Vector3f n_j = match.second.n;
        Eigen::Vector3f n_j_ = undoHomogeneousVector((params_->init_pose[id_i].inverse()*
                                                     params_->init_pose[id_j])*
                                                     getHomogeneousVector(n_j)); 

        Eigen::Vector3f rot_error = (n_i - n_j_);
        accum_error += rot_error.dot(rot_error);

        // From first to second
        rotationCov += n_j*n_i.transpose();
    }

    if (params_->verbose)
        std::cout << "Rotation error (initial): " << accum_error << std::endl;

    // Calculate calibration Rotation
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();

    if (params_->verbose)
    {
        std::cout << "Conditioning: " << conditioning;
        if (conditioning > 100)
            std::cout << " (bad)";
        std::cout << std::endl;
    }

    Eigen::Matrix3f rotation = svd.matrixV()*svd.matrixU().transpose();
    float det = rotation.determinant();
    if(det != 1)
    {
        Eigen::Matrix3f aux = Eigen::Matrix3f::Identity();
        aux(2, 2) = det;
        rotation = svd.matrixV()*aux*svd.matrixU().transpose();
    }

    accum_error = 0.f;
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        Eigen::Vector3f n_j_ = rotation*match.second.n;
        n_j_ = n_j_/n_j_.norm();

        Eigen::Vector3f rot_error = (n_i - n_j_);
        accum_error += rot_error.dot(rot_error);    //i*N + j - sum(k+1)_(k=0)^(i)
    }

    if (params_->verbose)
        std::cout << "Rotation error (after cal): " << accum_error << std::endl;

    return rotation;
}


Eigen::Vector3f Calibration::calibrateTranslation(std::size_t id_i, std::size_t id_j)
{
    // Calibration system
    Eigen::Matrix3f translationHessian = Eigen::Matrix3f::Zero();
    Eigen::Vector3f translationGradient = Eigen::Vector3f::Zero();

    std::size_t idx = params_->getPairId(id_i, id_j);
    Correspondences matches = getRandomElements(params_->lim_correspondences, all_matches[idx]);
    PlaneMatch match;

    float accum_error = 0.f;

    std::size_t N = matches.size();
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        float trans_error = match.second.d - match.first.d;

        translationHessian += (n_i*n_i.transpose());
        translationGradient += (n_i*trans_error);

        Eigen::Vector3f t ((params_->init_pose[id_i].inverse()*params_->init_pose[id_j]).block(0, 3, 3, 1));
        float r = trans_error - t.dot(n_i);
        accum_error += r*r;
    }

    if (params_->verbose)
        std::cout << "Translation error (initial): " << accum_error << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(translationHessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "FIM translation " << svd.singularValues().transpose() << std::endl;

    Eigen::Vector3f translation = translationHessian.inverse()*translationGradient;

    accum_error = 0.f;
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        float trans_error = match.second.d - match.first.d;

        float r = trans_error - translation.dot(n_i);
        accum_error += r*r;
    }

    if (params_->verbose) 
    {
        std::cout << "Translation error (after): " << accum_error << std::endl;
        std::cout << std::endl;
    }
    return translation;
}


Eigen::Matrix4f Calibration::runCalibrationSTD(std::size_t id_i, std::size_t id_j)
{
    Eigen::Matrix3f rot = this->calibrateRotation(id_i, id_j);      // Calibrate transformation matrix from coordinates of sensor with id_i (relative)
    Eigen::Vector3f t = this->calibrateTranslation(id_i, id_j);     // to system of sensor id_j (reference, should be sensor 0)

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block(0, 0, 3, 3) = rot;
    T.block(0, 3, 3, 1) = t;

    return T;
}


/*  Iterative Solutions */

Eigen::Matrix4f Calibration::runCalibrationGN(std::size_t id_i, std::size_t id_j, bool robust)
{
    // define initial value, and update increment [TODO: define initial value, otherwise 0]
    Vector6d X, Xinc;
    Matrix4d T, Tinc, Tgt;
    T = Tini;  //[TODO: T = Tini]

//    if available, load GT
//    Tgt = Matrix4d::Identity();
//    Tgt.block(0,0,1,4) <<	0.6316, -0.0318, 0.7746, 0.2558;
//    Tgt.block(1,0,1,4) <<	0.3198, 0.9209, -0.2229, 0.4039;
//    Tgt.block(2,0,1,4) <<	-0.7062, 0.3885, 0.5919, 0.3731;

    std::size_t idx = params_->getPairId(id_i, id_j);
    Correspondences matches = getRandomElements(params_->lim_correspondences, all_matches[idx]);

    // define hessians, gradients, and residuals
    int N = matches.size();
    Matrix6d Hi, H;
    Vector6d gi, g;
    Vector4d ei;
    double   e = 0.0, ei_sq;
    H   = Matrix6d::Zero();
    g   = Vector6d::Zero();

    // define GN/LM parameters
    double delta          = 0.00001f;
    double err_prev       = 9999999999999.9;
    int max_iters         = 30;
    double min_err        = 0.0000001;
    double min_err_change = 0.0000001;

    // start GN loop (if LM we have to first estimate the order of lambdaa)
    for( int i = 0; i < max_iters; i++ )
    {

        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;

        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
            // update Hessian, gradient (neg), and error
            H += J.transpose() * J  * w;
            g -= J.transpose() * ei * w;
            e += ei_sq * w;
        }

        // if the difference is very small stop
        if( ( abs(e-err_prev) < min_err_change ) || ( e < min_err ) )
            break;

        // update step
        LDLT<Matrix6d> solver(H);
        Xinc = solver.solve(g);
        T = T * expmap_se3(Xinc);

//        cout << endl << "Iteration " << i << "\t" << e;

        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( Xinc.norm() < numeric_limits<double>::epsilon() )
            break;

        // update previous values
        err_prev = e;

    }

    // show solution (DBG)
    Vector6d calib_err = logmap_se3( T * inverse_se3(Tgt) );
    cout << endl << endl;
    cout << "Final residue: " << err_prev << endl << endl << endl;
    cout << "Rotation error (rad)  = " << calib_err.tail(3).norm() << endl ;
    cout << "Translation error (m) = " << calib_err.head(3).norm() << endl << endl << endl;
//    cout << "Calibration: " << endl << "---------------------------------------" << endl << endl;
//    cout << T << endl << endl << endl;
//    cout << "Uncertainty matrix: " << endl << "---------------------------------------" << endl << endl;
//    cout << H.inverse() << endl << endl;

    return T.cast<float>();
}


Eigen::Matrix4f Calibration::runCalibrationLM(std::size_t id_i, std::size_t id_j, bool robust)
{

    // define initial value, and update increment [TODO: define initial value, otherwise 0]
    Vector6d X, Xinc;
    Matrix4d T, Tinc, Tgt;
    T = Tini;  //[TODO: T = Tini]

    // if available, load GT
//    Tgt = Matrix4d::Identity();
//    Tgt.block(0,0,1,4) <<	0.6316, -0.0318, 0.7746, 0.2558;
//    Tgt.block(1,0,1,4) <<	0.3198, 0.9209, -0.2229, 0.4039;
//    Tgt.block(2,0,1,4) <<	-0.7062, 0.3885, 0.5919, 0.3731;

    std::size_t idx = params_->getPairId(id_i, id_j);
    Correspondences matches = getRandomElements(params_->lim_correspondences, all_matches[idx]);

    // define hessians, gradients, and residuals
    int N = matches.size();
    Matrix6d Hi, H;
    Vector6d gi, g;
    Vector4d ei;
    double   e = 0.0, ei_sq;
    H   = Matrix6d::Zero();
    g   = Vector6d::Zero();

    // define GN/LM parameters
    double delta          = 0.00001;
    double err_prev       = 9999999999999.9;
    int max_iters         = 30;
    double min_err        = 0.0000001;
    double min_err_change = 0.0000001;
    double lambda         = 0.001; // we pre-calculate it later
    double lambda_k       = 5.0;
    bool precalculate_lambda = true;

    // pre-calculate lambda value
    if( precalculate_lambda )
    {
        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;
        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
        }
        // initial guess of lambda
        double Hmax = 0.0;
        for( int i = 0; i < 6; i++)
        {
            if( H(i,i) > Hmax || H(i,i) < -Hmax )
                Hmax = fabs( H(i,i) );
        }
        lambda = Hmax;
    }

    // start GN loop (if LM we have to first estimate the order of lambda)
    for( int i = 0; i < max_iters; i++ )
    {

        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;

        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
            // update Hessian, gradient (neg), and error
            H += J.transpose() * J  * w;
            g -= J.transpose() * ei * w;
            e += ei_sq * w;
        }

        // if the difference is very small stop
        if( ( abs(e-err_prev) < min_err_change ) || ( e < min_err ) )
            break;

        // update H, estimate increment
        for( int j = 0; j < 6; j++ )
            H(j,j) += lambda * H(j,j);
        LDLT<Matrix6d> solver(H);
        Xinc = solver.solve(g);

        // update LM
        if( e > err_prev )
        {
            lambda /= lambda_k;
        }
        else
        {
            lambda *= lambda_k;
            T = T * expmap_se3(Xinc);
        }
//        cout << endl << "Iteration " << i << "\t" << e;

        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( Xinc.norm() < numeric_limits<double>::epsilon() )
            break;

        // update previous values
        err_prev = e;
    }

    // show solution (DBG)
    Vector6d calib_err = logmap_se3( T * inverse_se3(Tgt) );
    cout << endl << endl;
    cout << "Final residue: " << err_prev << endl << endl << endl;
    cout << "Rotation error (rad)  = " << calib_err.tail(3).norm() << endl ;
    cout << "Translation error (m) = " << calib_err.head(3).norm() << endl << endl << endl;
    // cout << "Calibration: " << endl << "---------------------------------------" << endl << endl;
    // cout << T << endl << endl << endl;
    // cout << "Uncertainty matrix: " << endl << "---------------------------------------" << endl << endl;
    // cout << H.inverse() << endl << endl;

    return T.cast<float>();
}


std::vector<Eigen::Matrix4f> Calibration::runAllCalibrations() {
    std::vector<Eigen::Matrix4f> results;
    std::size_t N = params_->init_pose.size();
    std::string calib_strategy = params_->calib_strategy;
    std::string algorithm = params_->algorithm;

    std::size_t id_i;
    std::size_t id_j;

    auto runCalibrationFunc = [this, algorithm](std::size_t id_i, std::size_t id_j) -> Eigen::Matrix4f {
        if (algorithm == "std")
            return runCalibrationSTD(id_i, id_j);
        else if (algorithm == "gn")
            return runCalibrationGN(id_i, id_j, false);
        else if (algorithm == "gn_robust")
            return runCalibrationGN(id_i, id_j, true);
        else if (algorithm == "lm")
            return runCalibrationLM(id_i, id_j, false);
        else if (algorithm == "lm_robust")
            return runCalibrationLM(id_i, id_j, true);
        else {
            std::cerr << algorithm << " is not a valid calibration algorithm." << std::endl;
            std::cerr << "Feasible parameters: 'std', 'gn', 'gn_robust', 'lm', lm_robust'." << std::endl;
            return Eigen::Matrix4f::Identity();
        }
    };

    if (calib_strategy == "standard") {
        results.resize(N - 1); // One field for every sensor pair (0, 1), (0, 2), ...
        for (std::size_t k = 1; k < N; ++k) 
        {
            id_i = k;
            id_j = 0;
            results[k - 1].resize(4, 4);
            results[k - 1] = runCalibrationFunc(id_i, id_j);

            std::cout << "----- " << algorithm << ": Run calibration with sensor pair (" << id_i << ", " << id_j << ") -----" << std::endl;
            std::cout << results[k-1] << std::endl;
            std::cout << std::endl;
        }
    } else if (calib_strategy == "redundant") 
    {
        std::size_t iters = params_->error_analysis ? 25 : 1;
        std::vector<Eigen::Matrix4f> CL_vector(iters, Eigen::Matrix4f::Zero()); // Store individual closed loop errors
        Eigen::Matrix4f sum_CL = Eigen::Matrix4f::Zero();    // Store the sum of closed loop matrices
        Eigen::Matrix4f T_total;

        for (std::size_t i = 0; i < iters; ++i) 
        {
            results.resize(N * (N - 1) / 2); // One field for every possible sensor pair

            for (std::size_t k = 0; k < results.size(); ++k) 
            {
                params_->getIndicesFromPairId(k, id_i, id_j);
                if (!all_matches[k].empty())
                {
                    results[k].resize(4, 4);
                    results[k] = runCalibrationFunc(id_i, id_j);

                    std::cout << "----- " << algorithm << ": Run calibration with sensor pair (" << id_i << ", " << id_j << ") -----" << std::endl;
                    std::cout << results[k] << std::endl;
                    std::cout << std::endl;
                }
                else
                    std::cout <<"WARNING: No data for sensor pair (" << id_i << ", " << id_j << ")." << std::endl;
            }

            T_total = Eigen::Matrix4f::Identity();

            if (N > 2)
            {
                // Calculate error by closed loop
                bool last = false;
                size_t id1, id2;
                for (std::size_t l = 0; l < N; ++l) 
                {   
                    id1 = l;
                    id2 = last ? 0 : (l + 1);
                    std::size_t vec_idx = params_->getPairId(id1, id2);
                    if (!last)
                    {
                        T_total = T_total * results[vec_idx];
                    }
                    else
                    {
                        T_total = T_total * results[vec_idx].inverse();
                    }
                    std::cout << "- (" << id1 << ", " << id2 << ") -";

                    if (l == N - 2)
                        last = true;
                }
                std::cout << std::endl;

                sum_CL += T_total;
                CL_vector[i] = T_total;
            }
            else 
            {
                std::cout << "WARNING: At least 3 sensors are needed for a closed-loop accuracy test. " << N << " were given." << std::endl;
            }
        }

        if (params_->error_analysis)
        {
            std::cout << "Mean closed-loop matrix:" << std::endl;
            Eigen::Matrix4f mean_CL = sum_CL / iters;
            std::cout << mean_CL << std::endl;

            // Extract rotation error in degrees and translation error in meters
            Eigen::Matrix3f mean_rotation_matrix = mean_CL.block<3,3>(0,0);
            float mean_rotation_error = (mean_rotation_matrix - Eigen::Matrix3f::Identity()).norm();
            std::cout << "Mean rotation error (norm): " << mean_rotation_error << std::endl;

            std::vector<float> angles = extractEulerAngles(mean_rotation_matrix);

            std::cout << "Angle errors:" << std::endl;
            std::cout << "\tdeg(x) = " << angles[0] << std::endl;
            std::cout << "\tdeg(y) = " << angles[1] << std::endl;
            std::cout << "\tdeg(z) = " << angles[2] << std::endl;

            float mean_translation_error = mean_CL.block<3,1>(0,3).norm();
            std::cout << "Mean translation error (meters): " << mean_translation_error << std::endl;

            std::vector<float> squared_diffs_R;
            std::vector<float> squared_diffs_t;
            
            // Calc std deviation
            for (const auto& res : CL_vector)
            {
                float rot_diff2mean = (res.block<3,3>(0,0) - Eigen::Matrix3f::Identity()).norm() - mean_rotation_error;
                squared_diffs_R.push_back(rot_diff2mean * rot_diff2mean);
                float t_diff2mean = (res.block<3,1>(0,3)).norm() - mean_translation_error;
                squared_diffs_t.push_back(t_diff2mean * t_diff2mean);
            } 
            // float sq_median_R = calculateMedian(squared_diffs_R);
            // std::cout << "Median rotation error: " << sqrt(sq_median_R) << std::endl;

            // float sq_median_t = calculateMedian(squared_diffs_t);
            // std::cout << "Median translation error: " << sqrt(sq_median_t) << std::endl;

            float variance_R_error = std::accumulate(squared_diffs_R.begin(), squared_diffs_R.end(), 0.0) / squared_diffs_R.size();
            float variance_t_error = std::accumulate(squared_diffs_t.begin(), squared_diffs_t.end(), 0.0) / squared_diffs_t.size();

            float stddev_R_err = sqrt(variance_R_error);
            float stddev_t_err = sqrt(variance_t_error);

            std::cout << "Rotation error standard deviation (norm): " << stddev_R_err << std::endl;
            std::cout << "Translation error standard deviation (meters): " << stddev_t_err << std::endl;
        }
        else if (!params_->error_analysis)
        {
            std::cout << "Closed loop transformation matrix:" << std::endl << T_total << std::endl;
        }
    } else {
        std::cerr << calib_strategy << " is not a valid calibration strategy. Set a feasible parameter ('redundant' or 'standard')." << std::endl;
    }

    return results;
}


double Calibration::robustKernel(double e)
{
    return 1.0 / ( 1.0 + e ) ;
}



