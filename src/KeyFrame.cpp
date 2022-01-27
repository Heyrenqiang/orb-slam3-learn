#include "../include/KeyFrame.hpp"
namespace ORBSLAM
{
    KeyFrame::KeyFrame(/* args */)
    {
    }

    KeyFrame::~KeyFrame()
    {
    }
    
    KeyFrame::KeyFrame(Frame &F) 
    {
        
    }
    
    void KeyFrame::Compute_bow() 
    {
        
    }
    
    void KeyFrame::Add_mappoint(MapPoint *p_mappoint,int &idx) 
    {
        
    }
    
    void KeyFrame::Update_connections() 
    {
        
    }

    set<MapPoint *> KeyFrame::Get_mappoints(){
        set<MapPoint*> s;
        for(int i=0;i<mvp_mappoints.size();i++){
            if(!mvp_mappoints[i]){
                continue;
            }
            MapPoint * p_mp = mvp_mappoints[i];
            if(!p_mp->Is_bad()){
                s.insert(p_mp);
            }
        }
        return s;
    }
    
}