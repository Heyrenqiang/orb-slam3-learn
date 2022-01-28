#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__
#include "../include/Map.hpp"
namespace ORBSLAM{
    class Map;
    class Optimizer
    {
    private:
        /* data */
    public:
        Optimizer(/* args */);
        ~Optimizer();
        void static Global_bundle_adjustment(Map *p_map,int iterations=5);
    };
    

    
}
#endif // __OPTIMIZER_H__