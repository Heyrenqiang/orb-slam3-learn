#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__
#include "../include/Typedef.hpp"
#include "../include/Frame.hpp"
#include "../include/MapPoint.hpp"
namespace ORBSLAM{
    class MapPoint;
    class Frame;
    class KeyFrame
    {
    private:
        /* data */
    public:
        KeyFrame(/* args */);
        ~KeyFrame();
        KeyFrame(Frame &F);
        void Compute_bow();
        void Add_mappoint(MapPoint *p_mappoint,int &idx);

        // vector<MapPoint *> mvp_mappoints;

    };
    

    
}
#endif // __KEYFRAME_H__