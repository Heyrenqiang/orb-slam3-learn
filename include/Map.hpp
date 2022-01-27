#ifndef __MAP_H__
#define __MAP_H__
#include "../include/KeyFrame.hpp"
namespace ORBSLAM{
    class KeyFrame;
    class Map
    {
    private:
        /* data */
    public:
        Map(/* args */);
        ~Map();
        void Add_key_frame(KeyFrame * KF);
    };

    
}
#endif // __MAP_H__