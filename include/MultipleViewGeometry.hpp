#ifndef __MULTIPLEVIEWGEOMETRY_H__
#define __MULTIPLEVIEWGEOMETRY_H__
#include "../include/Typedef.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM{
    class MultipleViewGeometry
    {
    private:
        /* data */
        float mf_sigma;
    public:
        MultipleViewGeometry(/* args */);
        ~MultipleViewGeometry();
        void Find_fundamental(vector<KeyPoint> &v_kps1,vector<KeyPoint> &v_kps2,vector<Match> &matches12,int &itration_num,vector<vector<int>> &candidates,Mat &F21,vector<bool> &vb_match_inline,int &ninliners);
        void Normalize(vector<Point2f> &v_p2f,vector<Point2f> &v_np2f,Mat &T);
        Mat ComputeF(vector<Point2f> &vp2f1,vector<Point2f> &vp2f2);
        float Check_fundamental(Mat &F21,vector<Point2f> &vp2f_1,vector<Point2f> &vp2f_2,vector<bool> vb_ifinliner,int &num_inline,float sigma);
    };
    

    
}
#endif // __MULTIPLEVIEWGEOMETRY_H__