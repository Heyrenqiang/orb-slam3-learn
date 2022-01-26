#ifndef __MULTIPLEVIEWGEOMETRY_H__
#define __MULTIPLEVIEWGEOMETRY_H__
#include "../include/Typedef.hpp"
#include "../include/Param.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class MultipleViewGeometry
    {
    private:
        /* data */
        Param *mp_param;
        float mf_sigma;
        Mat mm_camera_intrinsics;
        float mf_reproj_th;

    public:
        MultipleViewGeometry(/* args */);
        ~MultipleViewGeometry();
        void Set_param(Param *param);
        void Find_fundamental(vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<Match> &matches12, int &itration_num, vector<vector<int>> &candidates, Mat &F21, vector<bool> &vb_match_inline, int &ninliners);
        void Normalize(vector<Point2f> &v_p2f, vector<Point2f> &v_np2f, Mat &T);
        Mat ComputeF(vector<Point2f> &vp2f1, vector<Point2f> &vp2f2);
        float Check_fundamental(Mat &F21, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinliner, int &num_inline, float sigma);
        bool ReconstructF(int ninline,vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinline, Mat &F21, Mat &R21, Mat &t21, vector<Point3f> &vp3d, vector<bool> &vb_triangulated, float min_parallax, int min_triangulated);
        void DecomposeE(Mat &E21, Mat &R1, Mat &R2, Mat &t);
        int CheckRT(Mat &R, Mat &t, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinline, vector<Point3f> &vp3f, vector<bool> &vb_triangulated, float &parallax);
        void Triangulate(const Point2f &kp1, const Point2f &kp2, const Mat &P1, const Mat &P2, Mat &p3d);
    };

}
#endif // __MULTIPLEVIEWGEOMETRY_H__