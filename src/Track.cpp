#include "../include/Track.hpp"
namespace ORBSLAM
{

    Track::Track()
    {
        mp_multipleviewgeometry = new MultipleViewGeometry();
        mi_itration_num = 200;
    }
    Track::~Track()
    {
    }
    void Track::Track_frame()
    {
    }

    bool Track::Mono_initial_two_frame(vector<KeyPoint> &v_kps1, vector<KeyPoint> &v_kps2, vector<int> &v_matches12, Mat &R21, Mat &t21, vector<Point3f> &v_p3f, vector<bool> &v_triangulated)
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
        vector<bool> vb_inlineesH, vb_inlineesF;
        int ninlinersH, ninlinersF;
        Mat H21, F21;
        mp_multipleviewgeometry->Find_fundamental(v_kps1, v_kps2, matches12, mi_itration_num, candidates, F21, vb_inlineesF, ninlinersF);

        return true;
    }
}