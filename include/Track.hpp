#ifndef __TRACK_H__
#define __TRACK_H__
#include "../include/Typedef.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class Track
    {
    private:
        /* data */

    public:
        Track(int a);
        ~Track();
        void Track_frame();
        bool Mono_initial_two_frame(vector<KeyPoint> &v_kps1,vector<KeyPoint> &v_kps2,vector<int> &v_matches12,Mat &R21,Mat &t21,vector<Point3f> &v_p3f,vector<bool> &v_triangulated);
    };




}
#endif // __TRACK_H__