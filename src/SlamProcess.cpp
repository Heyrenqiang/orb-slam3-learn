#include "../include/SlamProcess.hpp"
namespace ORBSLAM
{
    SlamProcess::SlamProcess(/* args */)
    {
        mp_orbfeature = new ORBFeature(1.2, 8);
        mp_matcher = new Matcher();
        mp_matcher->Set_find_match_parameters(30, 0.5, true, 120);
        mp_tracker = new Track();
        mp_param = new Param();
        mp_tracker->Set_param(mp_param);

        mb_mono_initialized = false;
        mb_initial_frame_ready = false;
    }
    void SlamProcess::Process(const Mat &im)
    {
        if (!mb_mono_initialized)
        {
            if (!mb_initial_frame_ready)
            {
                mF_initial_frame = Frame(im, mp_orbfeature, 5000);
                // Frame m = mF_initial_frame;
                if (mF_initial_frame.mi_feature_num > 100)
                {
                    mvp_prematched.resize(mF_initial_frame.mi_feature_num);
                    for (int i = 0; i < mF_initial_frame.mi_feature_num; i++)
                    {
                        mvp_prematched[i] = mF_initial_frame.mv_orb_unkeypoints[i].pt;
                    }
                    mb_initial_frame_ready = true;

                    // im1 = im.clone();
                    return;
                }
            }
            else
            {
                mF_curframe = Frame(im, mp_orbfeature, 5000);
                if (mF_curframe.mi_feature_num <= 100)
                {
                    mb_initial_frame_ready = false;
                    return;
                }
                int nmateches = mp_matcher->Search_for_initialization(mF_initial_frame, mF_curframe, mvp_prematched, mvi_initial_matches, 100);
                cout<<"nmatches:"<<nmateches<<endl;
                Mat R21,t21;
                vector<bool> vb_triangulated;
                if(mp_tracker->Mono_initial_two_frame(mF_initial_frame.mv_orb_unkeypoints,mF_curframe.mv_orb_unkeypoints,mvi_initial_matches,R21,t21,mvp3f_initial3d,vb_triangulated)){
                    cout<<"yes............"<<endl;
                }
                // im2 = im.clone();

                // vector<DMatch> m;
                // m.reserve(mvi_initial_matches.size());
                // for (int i = 0; i < mvi_initial_matches.size(); i++)
                // {
                //     if (mvi_initial_matches[i] > 0)
                //     {
                //         m.push_back(DMatch(i, mvi_initial_matches[i], 5.0));
                //     }
                // }

                // cv::Mat img_goodmatch;
                // cv::drawMatches(im1, mF_initial_frame.mv_orb_keypoints, im2, mF_curframe.mv_orb_keypoints, m, img_goodmatch);

                // cv::imshow("good matches", img_goodmatch);
                // cv::waitKey(0);
                cout << "nmatches:" << nmateches << endl;
            }
        }
        else
        {
        }
    }
}