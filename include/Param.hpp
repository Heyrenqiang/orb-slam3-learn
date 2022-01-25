#ifndef __PARAM_H__
#define __PARAM_H__
#include "../include/Typedef.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class Param
    {
    public:
        /* data */
        Mat mm_distcoef;
        Mat mm_camera_intrinsics;

    public:
        Param(/* args */);
        ~Param();
        void Set_camera_param();
    };
}
#endif // __PARAM_H__