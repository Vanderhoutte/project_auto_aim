#ifndef AXIS_CALCULATOR_HPP
#define AXIS_CALCULATOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include "amor_detector.hpp"

cv::Point2f calculate_axis(const armorPose& former,
    const armorPose& latest)
{
const float dt = 0.0167f;  // 时间间隔
cv::Mat p0_mat = former.position;
cv::Mat p1_mat = latest.position;
std::cout << "armor_poses_1:" << p0_mat << std::endl << former.rotation << std::endl;
std::cout << "armor_poses_2:" << p1_mat << std::endl << latest.rotation << std::endl;
// —— 1. 取位置向量（假设 position 是 3×1 Mat）
p0_mat.convertTo(p0_mat,CV_32F);
p1_mat.convertTo(p1_mat,CV_32F);
cv::Vec3f p0(
p0_mat.at<float>(0, 0),
p0_mat.at<float>(0, 1),
p0_mat.at<float>(0, 2)
);
std::cout << "p0: " << p0 << std::endl;
cv::Vec3f p1(
p1_mat.at<float>(0, 0),
p1_mat.at<float>(0, 1),
p1_mat.at<float>(0, 2)
);
std::cout << "p1: " << p1 << std::endl;

// —— 2. 相对旋转
cv::Mat Rrel = latest.rotation * former.rotation.inv();
Rrel.convertTo(Rrel, CV_32F);
cv::Vec3f rvec;
cv::Rodrigues(Rrel, rvec);
float theta = cv::norm(rvec);
std::cout << "theta: " << theta << std::endl;

// —— 3. 单位旋转轴
cv::Vec3f axis_unit;
if (theta < 1e-6f) {
// 退化到直线运动：随便给个轴（不影响后面直线分支返回）
axis_unit = cv::Vec3f(0,1,0);
} else {
axis_unit = rvec / theta;
}
std::cout << "axis_unit: " << axis_unit << std::endl;

// —— 4. 角速度和线速度
cv::Vec3f omega = axis_unit * (theta / dt);
cv::Vec3f v     = (p1 - p0) / dt;
std::cout << "delta" << p1-p0 << std::endl;
float omega_norm = cv::norm(omega);
float v_norm     = cv::norm(v);
float omega_y = std::abs(axis_unit[1] * (theta / dt));
float r       = v_norm / omega_y;
std::cout << "omega_norm: " << omega_norm
<< ", v_norm: " << v_norm << std::endl;

// 如果角速度很小，就认为是直线运动，直接返回中点
cv::Point2f p0_xz{ p0[0], p0[2] };
cv::Point2f p1_xz{ p1[0], p1[2] };
if (omega_norm < 1e-6f) {
cv::Point2f mid = (p0_xz + p1_xz) * 0.5f;
std::cout << "直线运动，center = " << mid << std::endl;
return mid;
}

// —— 5. 曲率半径 r = |v| / |ω|
//float r = v_norm / omega_norm;
std::cout << "radius r: " << r << std::endl;

// —— 6. 计算弦长 L（在 XZ 平面上）
float L = cv::norm(p1_xz - p0_xz);
std::cout << "chord length L: " << L << std::endl;

// —— 7. 弦心距 h = sqrt(r^2 - (L/2)^2)
float halfL = L * 0.5f;
float delta = r*r - halfL*halfL;
if (delta < 0) {
std::cout << "delta < 0, clamp to 0 (数值误差或直线)" << std::endl;
delta = 0;
}
float h = std::sqrt(delta);
std::cout << "chord-to-center h: " << h << std::endl;

// —— 8. 弦的中点
cv::Point2f mid = (p0_xz + p1_xz) * 0.5f;

// —— 9. 法线方向
cv::Point2f dir = p1_xz - p0_xz;
cv::Point2f normal{-dir.y, dir.x};
float nn = std::hypot(normal.x, normal.y);
if (nn > 1e-6f) {
normal *= (1.0f/nn);
}
if (normal.y < 0) {
normal = -normal;
}
std::cout << "normal: " << normal << std::endl;

// —— 10. 圆心
cv::Point2f center = mid + normal * h;
std::cout << "circle center: " << center << std::endl;
return center;
}

bool compare_armorPose(armorPose first,armorPose second)
{
    std::cout<< "compare_armorPose" << std::endl;
    cv::Mat diff;
    cv::compare(first.position, second.position, diff,cv::CMP_NE);
    if (cv::countNonZero(diff) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<cv::Point2f> calculate_RotatingShaft(const std::vector<armorPose> former,const std::vector<armorPose> latest) 
{
    if(former.empty() || latest.empty())
    {
        std::cout << "former or latest empty" << std::endl;
        return std::vector<cv::Point2f>{};
    }
    std::cout << "sizes: " << former.size() << " " << latest.size() << std::endl;
    armorPose best_former;
    double min_distance = 100000;
    double distance;
    std::vector<cv::Point2f> axiss;
    std::cout << "init_completed" << std::endl;
    for(armorPose armor_now: latest)
    {
        
        for(armorPose armor_former: former)
        {
            
            
                distance = cv::norm(armor_now.position - armor_former.position);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    best_former = armor_former;
                }
            
        }
        std::cout << "best_former_fetched" << std::endl;
        std::cout << "best_former:" << best_former.position << std::endl;
        axiss.push_back(calculate_axis(best_former,armor_now));

    }
    if(axiss.empty())
    {
        return std::vector<cv::Point2f>{};
    }
    if(axiss.size() == 1)
    {
        return std::vector<cv::Point2f>{axiss[0]};
    }
    for(auto axis1: axiss)
    {
        for(auto axis2: axiss)
        {
            if(axis1 == axis2)
            {
                continue;
            }
            else
            {
                if(cv::norm(axis1 - axis2) < 0.2)
                {
                    auto it = (std::find(axiss.begin(), axiss.end(), axis2));
                    if (it != axiss.end()) {
                        axiss.erase(it);
                    }
                    else
                    {
                        axiss.pop_back();
                    }
                }
            }
        }
    }

    return axiss;
}

#endif