#include "../include/Param.hpp"
namespace ORBSLAM
{
    Param::Param(/* args */)
    {
        Set_camera_param();
    }

    Param::~Param()
    {
    }

    void Param::Set_camera_param()
    {
        mm_camera_intrinsics = Mat::eye(3, 3, CV_32F);

        float fx = 458.654;
        float fy = 457.296;
        float cx = 367.215;
        float cy = 248.375;

        mm_camera_intrinsics.at<float>(0, 0) = fx;
        mm_camera_intrinsics.at<float>(0, 2) = cx;
        mm_camera_intrinsics.at<float>(1, 1) = fy;
        mm_camera_intrinsics.at<float>(1, 2) = cy;

        float k1 = -0.28340811;
        float k2 = 0.07395907;
        float p1 = 0.00019359;
        float p2 = 1.76187114e-05;

        mm_distcoef = Mat::zeros(4, 1, CV_32F);
        mm_distcoef.at<float>(0) = k1;
        mm_distcoef.at<float>(1) = k2;
        mm_distcoef.at<float>(2) = p1;
        mm_distcoef.at<float>(3) = p2;
    }
}