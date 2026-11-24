#include "../include/amor_detector.hpp"
#include "../include/kalman.hpp"
#include "../include/quaternion.hpp"
#include "../include/pose.hpp"
#include "../include/axis_calculator.hpp"
#include <iostream>
#include <fstream>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>

/* 
distortion_coefficients: [-0.04313325802537415, 0.3598873080850437, -0.011789027160352577, -0.0068734187976891474, 0.0]
rune_camera_matrix: [2142.4253006101626, 0.0, 644.8800557555103, 0.0, 2139.740720699495, 547.26009197675802, 0.0, 0.0, 1.0] 
*/
cv::Mat test_camera_matrix = (cv::Mat_<double>(3, 3) <<
    2142.4253006101626, 0.0, 644.8800557555103, 0.0, 2139.740720699495, 547.26009197675802, 0.0, 0.0, 1.0);
cv::Mat test_distortion_coefficients = (cv::Mat_<double>(1, 5) << 
    -0.04313325802537415, 0.3598873080850437, -0.011789027160352577, -0.0068734187976891474, 0.0);






int main()
{
    bool flag_kalinit = false;
    bool flag_armor_init = false;
    kalman kf_x(2,1);
    kalman kf_y(2,1);
    Eigen::MatrixXd z = Eigen::MatrixXd::Zero(1,1);
    Eigen::MatrixXd x0(2,1);
    kf_x.elementChange_P(5);
    kf_y.elementChange_P(5);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2,1);
    Eigen::MatrixXd H(1,2); H << 1,0;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1,1) * 0.01;
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
    std::ofstream output_file("armor_pose_results.txt");
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file!" << std::endl;
        return -1;
    }
    std::vector<armorPose> armor_poses;
    std::vector<armorPose> former_armor_poses;
    std::vector<cv::Point2f> axiss;
    for(int i = 0;i < 1031;i++)
    {
        std::string image_path = "../frames/frame_" + std::to_string(i) + ".jpg";
        cv::Mat src = cv::imread(image_path);
        if (src.empty())
        {
            std::cerr << "Error: Could not load image!" << std::endl;
            return -1;
        }

        armor_poses = armor_pose_calculator(
            src,
            true,
            test_camera_matrix,
            test_distortion_coefficients,
            false);
        std::cout << "############################################################" << std::endl;
        std::cout << "Frame: " << i << std::endl;
        output_file << "############################################################" << std::endl;
        output_file << "Frame: " << i << std::endl;
        if(armor_poses.empty())
        {
            std::cout << "Armor_Not_Found" << std::endl;
            output_file << "Armor_Not_Found" << std::endl;
        }
        else
        {
            std::cout << "Armor_Found" << std::endl;
            output_file << "Armor_Found" << std::endl;
            for(auto& armor:armor_poses)
            {
                std::cout << "Armor position: " << armor.position << std::endl;
                output_file << "Armor position: " << armor.position << std::endl;
                std::cout << "Armor rotation: " << armor.rotation << std::endl;
                output_file << "Armor rotation: " << armor.rotation << std::endl;
            }
            if (!flag_armor_init&&!armor_poses.empty())
            {
                flag_armor_init = true;
                former_armor_poses = armor_poses;
                continue;
            }
            if(armor_poses.empty())
            {
                continue;
            }
            else
            {
                if(former_armor_poses.empty())
                {
                    former_armor_poses = armor_poses;
                    continue;
                }
                std::cout << "Calculate_RotatingShaft" << std::endl;
                output_file << "Calculate_RotatingShaft" << std::endl;
                axiss = calculate_RotatingShaft(former_armor_poses,armor_poses);
                if(axiss.empty())
                {
                    std::cout << "Axis_Not_Found" << std::endl;
                    output_file << "Axis_Not_Found" << std::endl;
                }
                else
                {
                    std::cout << "Axis_Found" << std::endl;
                    output_file << "Axis_Found" << std::endl;
                    for(auto& axis:axiss)
                    {
                        if(std::abs(axis.x) > 10||std::abs(axis.y) > 10)
                        {
                            auto it = std::find(axiss.begin(), axiss.end(), axis);
                            if (it != axiss.end()) {
                                axiss.erase(it);
                            }
                            else
                            {
                                axiss.pop_back();
                            }
                        }
                        std::cout << "Axis: " << axis << std::endl;
                        output_file << "Axis: " << axis << std::endl;
                    }
                }
            }
            former_armor_poses = armor_poses;
        }
        /*if(!flag_kalinit&&axiss.size() > 0)
        {
            flag_kalinit = true;
            x0 << axiss[0].x,axiss[0].y;
            kf_x.elementChange_X(x0);
            x0 << axiss[1].x,axiss[1].y;
            kf_y.elementChange_X(x0);
            std::cout << "Kalman_Init" << std::endl;
            output_file << "Kalman_Init" << std::endl;
        }
        else if(axiss.size() > 0)
        {
            std::cout << "Kalman_Update" << std::endl;
            output_file << "Kalman_Update" << std::endl;
            for(auto &axis:axiss)
            {
                z << axis.x;
                kf_x.update(H,z,R);
                z << axis.y;
                kf_y.update(H,z,R);

            }
            
        }*/



    }
    std::cout << "############################################################" << std::endl;
    output_file << "############################################################" << std::endl;
    output_file.close();
    std::cout << "Results saved to armor_pose_results.txt" << std::endl;
    return 0;
}