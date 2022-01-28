#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__
#include "../include/Typedef.hpp"
#include "../include/Frame.hpp"
#include "../include/MapPoint.hpp"
using namespace std;
using namespace cv;
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
        void Update_connections();
        set<MapPoint *> Get_mappoints();
        float Compute_scene_median_depth(int q);
        int Tracked_mappoints(const int &minobs);
        Mat Get_pose();
        Mat Get_poseinv();
        void Set_pose(Mat &tcw);
        vector<MapPoint*> Get_vect_mappoints();
        
        


        vector<MapPoint *> mvp_mappoints;
        KeyFrame * mpKF_pre;
        KeyFrame * mpKF_next;

    };
    

    
}
#endif // __KEYFRAME_H__