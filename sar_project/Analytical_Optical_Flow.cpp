#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>  // Include OpenCV for image handling
#include <fstream>  // Include this for file I/O

// Function to save Eigen matrix as CSV
void saveMatrixAsCSV(const Eigen::MatrixXd& matrix, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << std::fixed << std::setprecision(2) << matrix(i, j);  // Format to 2 decimal places
                if (j != matrix.cols() - 1) {
                    file << ",";  // Add a comma between elements
                }
            }
            file << "\n";  // New line at the end of each row
        }
        file.close();
        std::cout << "Matrix saved as " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
}

// Function to save Eigen vector as CSV
void saveVectorAsCSV(const Eigen::VectorXd& vector, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (int i = 0; i < vector.size(); ++i) {
            file << std::fixed << std::setprecision(2) << vector(i);  // Format to 2 decimal places
            if (i != vector.size() - 1) {
                file << ",";  // Add a comma between elements
            }
        }
        file << "\n";  // New line at the end
        file.close();
        std::cout << "Vector saved as " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
}

int main() {

    // Load the current and previous images using OpenCV
    std::string cur_img_path = "/home/dlee/ros2_ws/src//sar_simulation/sar_project/Generated_Cur_img_1.png";
    std::string prev_img_path = "/home/dlee/ros2_ws/src//sar_simulation/sar_project/Generated_Prev_img_1.png";
    
    // Load images as grayscale (single channel)
    cv::Mat cur_img_cv = cv::imread(cur_img_path, cv::IMREAD_GRAYSCALE);
    cv::Mat prev_img_cv = cv::imread(prev_img_path, cv::IMREAD_GRAYSCALE);

    // Check if images are loaded successfully
    if (cur_img_cv.empty()) {
        std::cerr << "Error: Could not load current image at " << cur_img_path << std::endl;
        return -1;
    }
    if (prev_img_cv.empty()) {
        std::cerr << "Error: Could not load previous image at " << prev_img_path << std::endl;
        return -1;
    }

    // Resize the images to 32x32 pixels if needed
    cv::resize(cur_img_cv, cur_img_cv, cv::Size(32, 32));
    cv::resize(prev_img_cv, prev_img_cv, cv::Size(32, 32));

    int HEIGHT_PIXELS = cur_img_cv.rows;
    int WIDTH_PIXELS = cur_img_cv.cols;
    int O_up = WIDTH_PIXELS / 2;
    int O_vp = HEIGHT_PIXELS / 2;

    // Convert OpenCV Mat to Eigen Matrix for cur_img and prev_img
    Eigen::MatrixXd cur_img(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd prev_img(HEIGHT_PIXELS, WIDTH_PIXELS);

    for (int v = 0; v < HEIGHT_PIXELS; ++v) {
        for (int u = 0; u < WIDTH_PIXELS; ++u) {
            cur_img(v, u) = static_cast<double>(cur_img_cv.at<uchar>(v, u));
            prev_img(v, u) = static_cast<double>(prev_img_cv.at<uchar>(v, u));
        }
    }

    // Save cur_img as a CSV file
    saveMatrixAsCSV(cur_img, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/cur_img.csv");
    saveMatrixAsCSV(prev_img, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/prev_img.csv");

    // Additional code for the image processing
    int I_0 = 255; //Brightness value (0-255)
    float FoV = 82.22; //Field of View [deg]

    float w = 3.6e-6;
    float f = 0.066e-3;
    float delta_t = 0.01667;

    Eigen::Matrix<double, 1, 3> Ku_1;
    Ku_1 << -1, 0, 1;
    Eigen::Matrix<double, 3, 1> Ku_2;
    Ku_2 << 1, 2, 1;

    Eigen::Matrix<double, 1, 3> Kv_1;
    Kv_1 << 1, 2, 1;
    Eigen::Matrix<double, 3, 1> Kv_2;
    Kv_2 << -1, 0, 1;

    Eigen::MatrixXd G_up = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_vp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_rp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_tp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);

    // Calculate image gradients
    for (int v_p = 1; v_p < HEIGHT_PIXELS - 1; ++v_p) {
        for (int u_p = 1; u_p < WIDTH_PIXELS - 1; ++u_p) {
            Eigen::MatrixXd patch = cur_img.block<3, 3>(v_p - 1, u_p - 1);

            G_vp(v_p, u_p) = (Ku_1 * patch).dot(Ku_2);
            G_up(v_p, u_p) = (Kv_1 * patch).dot(Kv_2);

            G_rp(v_p, u_p) = (2 * (u_p - O_up) + 1) * G_up(v_p, u_p) + (2 * (v_p - O_vp) + 1) * G_vp(v_p, u_p);
            G_tp(v_p, u_p) = cur_img(v_p, u_p) - prev_img(v_p, u_p);
        }
    }
    saveMatrixAsCSV(G_up, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/G_up_matrix.csv");
    saveMatrixAsCSV(G_vp, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/G_vp_matrix.csv");
    saveMatrixAsCSV(G_rp, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/G_rp_matrix.csv");
    saveMatrixAsCSV(G_tp, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/G_tp_matrix.csv");

    Eigen::MatrixXd X(3, 3);
    X(0, 0) = f * G_vp.array().square().sum();
    X(0, 1) = -f * (G_up.array() * G_vp.array()).sum();
    X(0, 2) = -(w / 2) * (G_rp.array() * G_vp.array()).sum();

    X(1, 0) = f * (G_vp.array() * G_up.array()).sum();
    X(1, 1) = -f * G_up.array().square().sum();
    X(1, 2) = -(w / 2) * (G_rp.array() * G_up.array()).sum();

    X(2, 0) = f * (G_vp.array() * G_rp.array()).sum();
    X(2, 1) = -f * (G_up.array() * G_rp.array()).sum();
    X(2, 2) = -(w / 2) * G_rp.array().square().sum();

    Eigen::Vector3d y;
    y(0) = (G_tp.array() * G_vp.array()).sum();
    y(1) = (G_tp.array() * G_up.array()).sum();
    y(2) = (G_tp.array() * G_rp.array()).sum();
    y *= (8 * w / delta_t);

    // Use JacobiSVD to calculate pseudo-inverse manually
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd S = svd.singularValues().asDiagonal();
    Eigen::MatrixXd S_inv = S;
    for (int i = 0; i < S.rows(); ++i) {
        if (S(i, i) > 1e-6) {
            S_inv(i, i) = 1.0 / S(i, i);  // Inverse of non-zero singular values
        } else {
            S_inv(i, i) = 0;  // Zero out small singular values to avoid instability
        }
    }

    Eigen::MatrixXd pinv_X = svd.matrixV() * S_inv * svd.matrixU().transpose();  // Pseudo-inverse calculation

    Eigen::Vector3d b = pinv_X * y;

    // Output the solution vector b
    std::cout << "Solution vector b: " << b.transpose() << std::endl;
    //std::cout << "Solution vector b: " << 1/b[2] << std::endl;

    // Save X matrix as a CSV file
    saveMatrixAsCSV(X, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/X_matrix.csv");

    // Save y vector as a CSV file
    saveVectorAsCSV(y, "/home/dlee/ros2_ws/src/sar_simulation/sar_project/y_vector.csv");

    return 0;
}
