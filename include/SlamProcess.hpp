#ifndef __SLAMPROCESS_H__
#define __SLAMPROCESS_H__
#include "../include/Typedef.hpp"
#include "../include/Frame.hpp"
#include "../include/ORBFeature.hpp"
#include "../include/Matcher.hpp"
#include "../include/Track.hpp"
#include "../include/Param.hpp"
#include "../include/Frame.hpp"
#include "../include/Map.hpp"
#include "../include/MapAtlas.hpp"
#include "../include/MapPoint.hpp"
#include "../include/Optimizer.hpp"
#include "../include/LocalMap.hpp"
#include <DBoW3/DBoW3.h>
#include "../include/Converter.hpp"
#include "../include/MapAtlas.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    class Optimizer;
    class SlamProcess
    {
    public:
        enum e_tracking_state
        {
            NOT_INITIALIZED = 1,
            OK = 2
        };

    private:
        /* data */
        ORBFeature *mp_orbfeature;
        Matcher *mp_matcher;
        Track *mp_tracker;
        Param *mp_param;
        Map *mp_map;
        MapAtlas *mp_mapatlas;
        LocalMap *mp_localmap;
        DBoW3::Vocabulary *mp_vocaburary;

        bool mb_mono_initialized;
        bool mb_initial_frame_ready;
        Frame mF_preframe;
        Frame mF_curframe;
        Frame mF_initial_frame;
        vector<Point2f> mvp_prematched;
        vector<int> mvi_initial_matches;
        vector<Point3f> mvp3f_initial3d;
        vector<KeyFrame *> mvp_keyframe;
        e_tracking_state me_state;

        // Mat im1, im2;

    public:
        SlamProcess(/* args */);

        void Process(const Mat &im);
        void Create_momo_initial_map();
        void Reset_active_map();
        void Compute_bow(KeyFrame* pkf);
    };

}
#endif // __SLAMPROCESS_H__