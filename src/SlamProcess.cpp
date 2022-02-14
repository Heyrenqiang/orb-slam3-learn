#include "../include/SlamProcess.hpp"
// #define _DEBUG
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
        std::string filepath = "/home/heyrenqiang/vscode/test/data/ORBvoc.txt";
        mp_vocaburary = new DBoW3::Vocabulary();
        // mp_vocaburary->load(filepath);
        mp_mapatlas = new MapAtlas();

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
                cout << "nmatches:" << nmateches << endl;
#ifdef _DEBUG
                vector<DMatch> m;
                m.reserve(mvi_initial_matches.size());
                for (int i = 0; i < mvi_initial_matches.size(); i++)
                {
                    if (mvi_initial_matches[i] > 0)
                    {
                        m.push_back(DMatch(i, mvi_initial_matches[i], 5.0));
                    }
                }

                cv::Mat img_goodmatch;
                cv::drawMatches(mF_initial_frame.m_image, mF_initial_frame.mv_orb_keypoints, mF_curframe.m_image, mF_curframe.mv_orb_keypoints, m, img_goodmatch);

                cv::imshow("good matches", img_goodmatch);
                cv::waitKey(0);
                // cout << "nmatches:" << nmateches << endl;
#endif
                Mat R21, t21;
                vector<bool> vb_triangulated;
                if (mp_tracker->Mono_initial_two_frame(mF_initial_frame.mv_orb_unkeypoints, mF_curframe.mv_orb_unkeypoints, mvi_initial_matches, R21, t21, mvp3f_initial3d, vb_triangulated))
                {
                    for (int i = 0; i < mvi_initial_matches.size(); i++)
                    {
                        if (mvi_initial_matches[i] >= 0 && !vb_triangulated[i])
                        {
                            mvi_initial_matches[i] = -1;
                            nmateches--;
                        }
                    }
                    mF_initial_frame.Set_pose(Mat::eye(4, 4, CV_32F));
                    Mat Tcw = Mat::eye(4, 4, CV_32F);
                    R21.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                    t21.copyTo(Tcw.rowRange(0, 3).col(3));
                    mF_curframe.Set_pose(Tcw);
                    Create_momo_initial_map();
                    cout << "yes............" << endl;
                }
            }
        }
        else
        {
        }
    }

    void SlamProcess::Create_momo_initial_map()
    {
        mp_mapatlas->Creat_map();
        Map * cur_map = mp_mapatlas->Get_current_map();
        KeyFrame *pkF_ini = new KeyFrame();
        KeyFrame *pkF_cur = new KeyFrame();
        swap(*((Frame *)pkF_ini), mF_initial_frame);
        swap(*((Frame *)pkF_cur), mF_curframe);
        pkF_ini->Init_keyframe();
        pkF_cur->Init_keyframe();

        pkF_ini->Set_map(cur_map);
        pkF_cur->Set_map(cur_map);
        Compute_bow(pkF_ini);
        Compute_bow(pkF_cur);
        cur_map->Add_key_frame(pkF_ini);
        cur_map->Add_key_frame(pkF_cur);
        for (int i = 0; i < mvi_initial_matches.size(); i++)
        {
            if (mvi_initial_matches[i] < 0)
            {
                continue;
            }
            Mat world_pose(mvp3f_initial3d[i]);
            MapPoint *p_mappoint = new MapPoint(world_pose);
            p_mappoint->Set_ref_keframe(pkF_cur);
            p_mappoint->Set_map(cur_map);
            pkF_ini->Add_mappoint(p_mappoint, i);
            pkF_cur->Add_mappoint(p_mappoint, mvi_initial_matches[i]);
            p_mappoint->Add_observation(pkF_ini, i);
            p_mappoint->Add_observation(pkF_cur, mvi_initial_matches[i]);
            p_mappoint->Compute_distinctive_descriptors(mp_matcher);
            p_mappoint->Update_normal_and_depth();
            // mF_curframe.mvp_mappoints[mvi_initial_matches[i]] = p_mappoint;
            // mF_curframe.mvb_outline[mvi_initial_matches[i]] = false;
            cur_map->Add_mappoint(p_mappoint);
        }

        pkF_ini->Update_connections();
        pkF_cur->Update_connections();
        // set<MapPoint*> sp_mappoints;
        // sp_mappoints = pkF_ini->Get_mappoints();
        Optimizer::Global_bundle_adjustment(cur_map, 20);
        float median_depth = pkF_ini->Compute_scene_median_depth(2);
        float inv_median_depth;
        inv_median_depth = 4.0f / median_depth;
        if (median_depth < 0 || pkF_cur->Tracked_mappoints(1) < 50)
        {
            Reset_active_map();
            return;
        }
        Mat Tcw = pkF_cur->Get_pose();
        Tcw.rowRange(0, 3).col(3) = Tcw.col(3).rowRange(0, 3) * inv_median_depth;
        pkF_cur->Set_pose(Tcw);

        vector<MapPoint *> vp_all_mappoints = pkF_ini->Get_vect_mappoints();
        for (int i = 0; i < vp_all_mappoints.size(); i++)
        {
            if (vp_all_mappoints[i])
            {
                MapPoint *p_mp = vp_all_mappoints[i];
                p_mp->Set_world_pose(p_mp->Get_world_pose() * inv_median_depth);
                p_mp->Update_normal_and_depth();
            }
        }
        pkF_cur->mpKF_pre = pkF_ini;
        pkF_ini->mpKF_next = pkF_cur;

        mp_localmap->Insert_keyframe(pkF_ini);
        mp_localmap->Insert_keyframe(pkF_cur);
        mvp_keyframe.push_back(pkF_cur);
        mvp_keyframe.push_back(pkF_ini);
        me_state = OK;
    }

    void SlamProcess::Reset_active_map()
    {
    }

    void SlamProcess::Compute_bow(KeyFrame *pkf)
    {
        vector<Mat> v_cur_desc = Converter::toDescriptorVector(pkf->mm_descriptors);
        mp_vocaburary->transform(v_cur_desc, pkf->mv_bowvector, pkf->mv_featurevector, 4);
        cout << "bow vector size:" << pkf->mv_bowvector.size() << endl;
        cout << "feature vector size:" << pkf->mv_featurevector.size() << endl;
    }

}