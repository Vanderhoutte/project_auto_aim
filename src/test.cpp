#include "../include/amor_detector.hpp"
#include "../include/kalman.hpp"
#include "../include/quaternion.hpp"
#include "../include/pose.hpp"
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
    std::ofstream output_file("armor_pose_results.txt");
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file!" << std::endl;
        return -1;
    }
    std::vector<armorPose> armor_poses;
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
        }       

    }
    std::cout << "############################################################" << std::endl;
    output_file << "############################################################" << std::endl;
    output_file.close();
    std::cout << "Results saved to armor_pose_results.txt" << std::endl;
    return 0;
}