#ifndef ORBFEATURE_H
#define ORBFEATURE_H
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/Frame.hpp"
using namespace cv;
using namespace std;
namespace ORBSLAM
{
    class Frame;
    class MyExtractorNode
    {
    public:
        MyExtractorNode() : bNoMore(false), feature_size(0) {}
        void DivideNode(MyExtractorNode &n1, MyExtractorNode &n2, MyExtractorNode &n3, MyExtractorNode &n4);

        vector<KeyPoint> vKeys;
        Point2i UL, UR, BL, BR;
        list<MyExtractorNode>::iterator lit;
        int feature_size;
        bool bNoMore;
    };

    class ORBFeature
    {
    public:
        // ORBFeature(int nfeatures,float scaleFactor,int nlevels,int iniThFAST,int minThFAST);
        ORBFeature(double scale, int nlevel);
        // int operator()(InputArray _image,InputArray _mask,vector<KeyPoint>& _keypoints,OutputArray _descriptors);

        vector<KeyPoint> DistribiteOctTree(const vector<KeyPoint> &vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY,
                                           const int &N);
        vector<KeyPoint> Test_DistribiteOctTree(const Mat &im, const vector<KeyPoint> &vToDistributeKeys, Rect &rect,
                                                const int &N);
        vector<KeyPoint> Test_DistribiteOctTree_by_prior_queue(const Mat &im, const vector<KeyPoint> &vToDistributeKeys, Rect &rect,
                                                               const int &N);
        void Extract_FASTFeature_by_grid(const Mat &img, const float grid_size, vector<KeyPoint> &vToDistributeKeys, Rect &rect);

        void compute_pyramid_scale_info(double scale_factor, int nlevel);
        vector<int> compute_xrange_by_y(int radius);
        void compute_feature_num_in_pyramids(int feature_num_to_extract, double scale_factor, int nlevel);
        vector<Mat> compute_pyramid_without_copyMakeBorder(const Mat &im, int nlevel);
        void compute_pyramid(const Mat &im, int nlevel);
        void compute_centroid_orientation(const Mat &im, vector<KeyPoint> &all_keypoints, vector<int> &xrange, int radius);
        void compute_orb_descriptor(const KeyPoint &kpt, const Mat &im, uchar *desc, int *pattern_array);
        void compute_descriptors(const Mat &im, vector<KeyPoint> &kepoints, Mat &descriptors);

        void extract_orb_compute_descriptor(const Mat &im, int feature_num, vector<KeyPoint> &orb_keypoints, Mat &descriptors);
        void undistort_keypoints(vector<KeyPoint> &keypoints,vector<KeyPoint> &undistoeted_keypoints,int feature_num);
        void compute_image_bounds(const Mat &im,float &box_minx,float &box_miny,float &box_maxx,float &box_maxy);


        vector<double> mvd_scale_factor;
        vector<double> mvd_inv_scale_factor;
        vector<double> mvd_level_sigma2;
        vector<double> mvd_inv_level_sigma2;
        vector<int> mvi_feature_num_in_each_pyrimad;
        vector<Mat> mvm_image_pyramid;
        vector<int> mvi_xrange;
        int mi_nlevel;
        double md_scale_factor;
        const int border = 16;
        const int grid_size = 35;
        const int half_patch_size = 15;
        const int patch_size = 31;
 

        const float factorPI = (float)(CV_PI / 180.f);
        const int edge_threshold = 19;

        Mat mm_distcoef;
        Mat mm_camera_intrinsics;



    };
}

#endif