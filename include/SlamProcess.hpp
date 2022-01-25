#ifndef __SLAMPROCESS_H__
#define __SLAMPROCESS_H__
#include "../include/Typedef.hpp"
#include "../include/Frame.hpp"
#include "../include/ORBFeature.hpp"
#include "../include/Matcher.hpp"
#include "../include/Track.hpp"
#include "../include/Param.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class SlamProcess
    {
    private:
        /* data */
        ORBFeature *mp_orbfeature;
        Matcher *mp_matcher;
        Track *mp_tracker;
        Param *mp_param;


        bool mb_mono_initialized;
        bool mb_initial_frame_ready;
        Frame mF_preframe;
        Frame mF_curframe;
        Frame mF_initial_frame;
        vector<Point2f> mvp_prematched;
        vector<int> mvi_initial_matches;
        vector<Point3f> mvp3f_initial3d;

        // Mat im1, im2;

    public:
        SlamProcess(/* args */);

        void Process(const Mat &im);
    };




}
#endif // __SLAMPROCESS_H__