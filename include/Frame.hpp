#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include <mutex>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

namespace ORBSLAM
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class ORBFeature;

    class Frame
    {
    public:
        Frame();

        // Copy constructor.
        // Frame(const Frame &frame);

        // Constructor for Monocular cameras.
        Frame(const Mat &im, ORBFeature *orbfeature, int feature_num_to_extract);
        void assign_features_to_grid();
        bool Pos_in_grid(const KeyPoint &kp,int &row,int &col);
        vector<int> Get_candidate_points_to_match(Point2f &vp_prematched,int window_size,int min_level,int max_level);

        Mat m_image;
        int mi_feature_num;
        vector<KeyPoint> mv_orb_keypoints;
        Mat mm_descriptors;
        vector<KeyPoint> mv_orb_unkeypoints;

        vector<int> mv_grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        static float mf_box_minx;
        static float mf_box_miny;
        static float mf_box_maxx;
        static float mf_box_maxy;

        static float mf_grid_element_width_inv;
        static float mf_grid_element_height_inv;

        static bool mb_initial_image_bound;
    };

}

#endif
