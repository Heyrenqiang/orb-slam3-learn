#ifndef __MAPPOINT_H__
#define __MAPPOINT_H__
#include "../include/Typedef.hpp"
#include "../include/KeyFrame.hpp"
#include "../include/Map.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class KeyFrame;
    class Map;
    class MapPoint
    {
    private:
        /* data */
    public:
        MapPoint(/* args */);
        ~MapPoint();
        MapPoint(Mat &pos, KeyFrame *p_KF, Map *p_map);
        void Add_key_frame(KeyFrame *KF);
        void Add_observation(KeyFrame *p_KF, int idx);
        void Compute_distinctive_descriptors();
        void Update_normal_and_depth();
        bool Is_bad();
    };

}
#endif // __MAPPOINT_H__