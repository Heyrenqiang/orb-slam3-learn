#ifndef __MAPPOINT_H__
#define __MAPPOINT_H__
#include "../include/Typedef.hpp"
#include "../include/Frame.hpp"
#include "../include/Map.hpp"
#include "../include/Matcher.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class KeyFrame;
    class Map;
    class Matcher;
    class MapPoint
    {
    public:
        /* data */
        Mat mm_world_pose;
        KeyFrame * mp_ref_keyframe;
        Map * mp_map;
        map<KeyFrame*,int> mmap_observation;
        Mat mm_descrptor;
        int m_nobs;
        bool mb_bad;
        float mf_max_distance;
        float mf_min_distance;
        Mat mm_normal_vector;

    public:
        MapPoint(/* args */);
        ~MapPoint();
        MapPoint(Mat &pos);
        MapPoint(Mat &pos, KeyFrame *p_KF, Map *p_map);
        void Set_ref_keframe(KeyFrame* p_KF);
        void Set_map(Map* p_map);
        void Add_key_frame(KeyFrame *KF);
        void Add_observation(KeyFrame *p_KF, int idx);
        void Compute_distinctive_descriptors(Matcher * p_matcher);
        void Update_normal_and_depth();
        bool Is_bad();
        void Set_world_pose(const Mat &pos);
        Mat Get_world_pose();
    };

}
#endif // __MAPPOINT_H__