#include "../include/Track.hpp"
namespace ORBSLAM
{

    Track::Track()
    {
    }
    Track::~Track()
    {
    }

    void Track::Set_param(Param *param)
    {
        mp_param = param;
        mp_multipleviewgeometry = new MultipleViewGeometry();
        mp_multipleviewgeometry->Set_param(mp_param);
        mi_itration_num = 200;
    }
    void Track::Track_frame()
    {
    }

    bool Track::Mono_initial_two_frame(vector<KeyPoint> &v_kps1, vector<KeyPoint> &v_kps2, vector<int> &v_matches12, Mat &R21, Mat &t21, vector<Point3f> &vp3f, vector<bool> &vb_triangulated)
    {
        vector<Match> matches12;
        matches12.reserve(v_kps2.size());
        vector<bool> matched1;
        matched1.resize(v_kps1.size());
        for (int i = 0; i < v_matches12.size(); i++)
        {
            if (v_matches12[i] >= 0)
            {
                matches12.push_back(make_pair(i, v_matches12[i]));
                matched1[i] = true;
            }
            else
            {
                matched1[i] = false;
            }
        }
        int n = matches12.size();
        vector<int> all_indices;
        all_indices.resize(n);
        vector<int> avaliable_indices;
        for (int i = 0; i < n; i++)
        {
            all_indices[i] = i;
        }
        vector<vector<int>> candidates = vector<vector<int>>(mi_itration_num, vector<int>(8, 0));
        srand((unsigned)time(NULL));
        for (int it = 0; it < mi_itration_num; it++)
        {
            avaliable_indices = all_indices;

            for (int j = 0; j < 8; j++)
            {
                int randi = rand() % (avaliable_indices.size());
                int idx = avaliable_indices[randi];
                candidates[it][j] = idx;
                avaliable_indices[randi] = avaliable_indices.back();
                avaliable_indices.pop_back();
            }
        }

        vector<Point2f> vp2f_1, vp2f_2;
        vp2f_1.resize(matches12.size());
        vp2f_2.resize(matches12.size());
        for (int i = 0; i < matches12.size(); i++)
        {
            vp2f_1[i] = v_kps1[matches12[i].first].pt;
            vp2f_2[i] = v_kps2[matches12[i].second].pt;
        }

        vector<bool> vb_inlineesH, vb_inlineesF;
        int ninlinersH, ninlinersF;
        Mat H21, F21;
        mp_multipleviewgeometry->Find_fundamental(vp2f_1, vp2f_2, matches12, mi_itration_num, candidates, F21, vb_inlineesF, ninlinersF);
        cout << "ninlinersF:" << ninlinersF << endl;
        // cout << "F21:" << F21 << endl;
        mp_multipleviewgeometry->ReconstructF(ninlinersF,vp2f_1, vp2f_2, vb_inlineesF, F21, R21, t21, vp3f, vb_triangulated, 1.0, 50);
        cout<<"R21:"<<R21<<endl;
        cout<<"t21"<<t21<<endl;
        return true;
    }
}