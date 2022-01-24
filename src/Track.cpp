#include "../include/Track.hpp"
namespace ORBSLAM
{

    Track::Track(int a)
    {
    }
    Track::~Track()
    {
    }
    void Track::Track_frame()
    {
    }

    bool Track::Mono_initial_two_frame(vector<KeyPoint> &v_kps1, vector<KeyPoint> &v_kps2, vector<int> &v_matches12, Mat &R21, Mat &t21, vector<Point3f> &v_p3f, vector<bool> &v_triangulated)
    {
        return true;
    }
}