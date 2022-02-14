#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include <mutex>
#include <opencv2/opencv.hpp>
#include <DBoW3/DBoW3.h>
#include "../include/MapPoint.hpp"
#include "../include/Map.hpp"
using namespace std;
using namespace cv;

namespace ORBSLAM
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class ORBFeature;
    class MapPoint;
    class Frame
    {
    public:
        Frame();

        // Copy constructor.
        // Frame(const Frame &frame);

        // Constructor for Monocular cameras.
        Frame(const Mat &im, ORBFeature *orbfeature, int feature_num_to_extract);
        void assign_features_to_grid();
        bool Pos_in_grid(const KeyPoint &kp, int &row, int &col);
        vector<int> Get_candidate_points_to_match(Point2f &vp_prematched, int window_size, int min_level, int max_level);
        void Set_pose(Mat Tcw);

        Mat m_image;
        int mi_feature_num;
        vector<KeyPoint> mv_orb_keypoints;
        Mat mm_descriptors;
        vector<KeyPoint> mv_orb_unkeypoints;
        // vector<MapPoint *> mvp_mappoints;
        // vector<bool> mvb_outline;

        vector<int> mv_grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        static float mf_box_minx;
        static float mf_box_miny;
        static float mf_box_maxx;
        static float mf_box_maxy;

        static float mf_grid_element_width_inv;
        static float mf_grid_element_height_inv;

        static bool mb_initial_image_bound;

        static vector<double> mvd_scale_factor;
        static vector<double> mvd_inv_scale_factor;
        static vector<double> mvd_level_sigma2;
        static vector<double> mvd_inv_level_sigma2;
        static int mi_nlevel;

        Mat mm_Tcw;
        Mat mm_Rcw;
        Mat mm_tcw;
        Mat mm_Twc;
        Mat mm_Rwc;
        Mat mm_twc;
    };

    class KeyFrame : public Frame
    {
    public:
        KeyFrame();

        void Compute_bow();
        void Add_mappoint(MapPoint *p_mappoint, int &idx);
        void Update_connections();
        set<MapPoint *> Get_mappoints();
        float Compute_scene_median_depth(int q);
        int Tracked_mappoints(const int &minobs);
        Mat Get_pose();
        Mat Get_poseinv();
        void Set_pose(Mat &tcw);
        vector<MapPoint *> Get_vect_mappoints();
        void Set_map(Map *p_map);
        void Add_connection(KeyFrame *pKF, int weight);
        void Add_child(KeyFrame *pKF);
        void Update_best_covisibles();
        void Init_keyframe();

        vector<MapPoint *> mvp_mappoints;
        KeyFrame *mpKF_pre;
        KeyFrame *mpKF_next;
        Map *mp_map;
        DBoW3::BowVector mv_bowvector;
        DBoW3::FeatureVector mv_featurevector;
        bool mb_bad;
        static long unsigned int sm_curid;
        long unsigned int mi_id;

        map<KeyFrame *, int> mmap_KF_connection_weight;
        vector<KeyFrame *> mv_ordered_KF;
        vector<int> mv_ordered_weight;

        bool mb_first_connect;

        KeyFrame* mp_parent;
        set<KeyFrame*> msp_children;
    };

}

#endif
