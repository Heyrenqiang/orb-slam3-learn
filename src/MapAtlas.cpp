#include "../include/MapAtlas.hpp"
namespace ORBSLAM
{
    MapAtlas::MapAtlas(/* args */)
    {
    }

    MapAtlas::~MapAtlas()
    {
    }
    Map *MapAtlas::Get_current_map()
    {
        mp_current_map = new Map();
        return mp_current_map;
    }
    
    void MapAtlas::Add_mappoint(MapPoint * p_mappoint) 
    {
        
    }

    void MapAtlas::Add_key_frame(KeyFrame *KF)
    {
    }

    vector<KeyFrame*> MapAtlas::Get_all_keyframes(){

    }
}