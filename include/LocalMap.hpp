#ifndef __LOCALMAP_H__
#define __LOCALMAP_H__
#include "../include/KeyFrame.hpp"
namespace ORBSLAM{
    class LocalMap
    {
    private:
        /* data */
    public:
        LocalMap(/* args */);
        ~LocalMap();
        void Insert_keyframe(KeyFrame * p_keyframe);
    };
    

    
}
#endif // __LOCALMAP_H__