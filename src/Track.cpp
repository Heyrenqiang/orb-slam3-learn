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
        vector<Match> matches12;
        matches12.reserve(v_kps2.size());
        vector<bool> matched1;
        matched1.resize(v_kps1.size());
        for(int i=0;i<v_matches12.size();i++){
            if(v_matches12[i]>=0){
                matches12.push_back(make_pair(i,v_matches12[i]));
                matched1[i]=true;
            }else{
                matched1[i]=false;
            }
        }
        return true;
    }
}