#ifndef __MAP_H__
#define __MAP_H__
#include "../include/Frame.hpp"
#include "../include/MapPoint.hpp"
using namespace std;

namespace ORBSLAM{
    class KeyFrame;
    class MapPoint;
    class Map
    {
    public:
        /* data */
        set<KeyFrame*> msp_keyframe;
        set<MapPoint*> msp_mappoint;
        int mi_iniKF_id;
        KeyFrame * mp_iniKF;
    public:
        Map(/* args */);
        ~Map();
        void Add_key_frame(KeyFrame * pKF);
        void Add_mappoint(MapPoint* pmp);
    };

    
}
#endif // __MAP_H__