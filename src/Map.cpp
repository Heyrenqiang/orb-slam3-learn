#include "../include/Map.hpp"
namespace ORBSLAM
{

    Map::Map(/* args */)
    {
    }

    Map::~Map()
    {
    }

    void Map::Add_key_frame(KeyFrame *pKF)
    {
        if(msp_keyframe.empty()){
            mi_iniKF_id = pKF->mi_id;
            mp_iniKF = pKF;
        }
        msp_keyframe.insert(pKF);
    }

    void Map::Add_mappoint(MapPoint *pmp)
    {
        msp_mappoint.insert(pmp);
    }

}