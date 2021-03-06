#ifndef __MAPALTAS_H__
#define __MAPALTAS_H__
#include "../include/Map.hpp"
#include "../include/KeyFrame.hpp"
#include "../include/MapPoint.hpp"
namespace ORBSLAM
{
    class MapPoint;
    class KeyFrame;
    class MapAtlas
    {
    private:
        /* data */
    public:
        MapAtlas(/* args */);
        ~MapAtlas();
        void Add_key_frame(KeyFrame *KF);
        Map *Get_current_map();
        void Add_mappoint(MapPoint * p_mappoint);

        Map *mp_current_map;
    };

}
#endif // __MAPALTAS_H__