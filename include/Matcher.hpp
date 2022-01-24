#ifndef __MATCHER_H__
#define __MATCHER_H__
#include "../include/Frame.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class Matcher
    {
    private:
        /* data */
        int mi_th_low = 30;
        float mf_best_secondbest_ratio;
        bool mb_check_orientation;
        int mi_bin_len;

    public:
        Matcher(/* args */);

        int Search_for_initialization(Frame &F1, Frame &F2, vector<Point2f> &vp_prematched, vector<int> &vn_matches12, int window_size);
        int Compute_descriptor_distance(const Mat &descriptor1, const Mat &descriptor2);
        void Compute_three_maxima(vector<int> *histo, int histo_len, int &idx1, int &idx2, int &idx3);

        void Set_find_match_parameters(int th_low, float best_second_ratio, bool check_orientation, int bin_len);
    };


}
#endif // __MATCHER_H__