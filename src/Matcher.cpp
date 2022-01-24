#include "../include/Matcher.hpp"
namespace ORBSLAM
{
    Matcher::Matcher(/* args */) 
    {
        
    }

    int Matcher::Search_for_initialization(Frame &F1, Frame &F2, vector<Point2f> &vp_prematched, vector<int> &vn_matches12, int window_size)
    {
        int nmatches = 0;
        const float bin_factor = mi_bin_len / 360.0f;
        vn_matches12 = vector<int>(F1.mi_feature_num, -1);
        vector<int> vn_matches21(F2.mi_feature_num, -1);
        vector<int> v_matched_descriptors_distance(F2.mi_feature_num, INT_MAX);
        vector<int> rot_hist[mi_bin_len];
        for (int i = 0; i < mi_bin_len; i++)
        {
            rot_hist[i].reserve(500);
        }
        int F1_kp_size = F1.mv_orb_unkeypoints.size();
        // TMP
        // cout << "descriptors:" << F1.mm_descriptors.row(3392) << endl;

        for (int i1 = 0; i1 < F1_kp_size; i1++)
        {
            KeyPoint kp1 = F1.mv_orb_unkeypoints[i1];
            int level1 = kp1.octave;
            // TODO why
            if (level1 > 0)
            {
                continue;
            }

            vector<int> candidate_index_inF2 = F2.Get_candidate_points_to_match(vp_prematched[i1], window_size, level1, level1);
            if (candidate_index_inF2.empty())
            {
                continue;
            }
            Mat descriptors1 = F1.mm_descriptors.row(i1);
            int best_dist = INT_MAX;
            int second_best_dist = INT_MAX;
            int best_idx = -1;
            for (vector<int>::iterator it = candidate_index_inF2.begin(); it != candidate_index_inF2.end(); it++)
            {
                int i2 = *it;
                Mat descriptors2 = F2.mm_descriptors.row(i2);
                int dist = Compute_descriptor_distance(descriptors1, descriptors2);
                if (v_matched_descriptors_distance[i2] <= dist)
                {
                    continue;
                }
                if (dist < best_dist)
                {
                    second_best_dist = best_dist;
                    best_dist = dist;
                    best_idx = i2;
                }
                else if (dist < second_best_dist)
                {
                    second_best_dist = dist;
                }
            }
            if (best_dist <= mi_th_low)
            {
                if (best_dist < (float)second_best_dist * mf_best_secondbest_ratio)
                {
                    if (vn_matches21[best_idx] >= 0)
                    {
                        vn_matches12[vn_matches21[best_idx]] = -1;
                        nmatches--;
                    }
                    vn_matches12[i1] = best_idx;
                    vn_matches21[best_idx] = i1;
                    v_matched_descriptors_distance[best_idx] = best_dist;
                    nmatches++;
                    if (mb_check_orientation)
                    {
                        float rot = F1.mv_orb_unkeypoints[i1].angle - F2.mv_orb_unkeypoints[best_idx].angle;
                        if (rot < 0.0)
                        {
                            rot += 360.0f;
                        }
                        int bin = cvFloor(rot * bin_factor);
                        // if (bin == mi_bin_len)
                        // {
                        //     bin = 0;
                        // }
                        assert(bin >= 0 && bin < mi_bin_len);
                        rot_hist[bin].push_back(i1);
                    }
                }
            }
        }
        if (mb_check_orientation)
        {
            int idx1 = -1;
            int idx2 = -1;
            int idx3 = -1;
            Compute_three_maxima(rot_hist, mi_bin_len, idx1, idx2, idx3);

            for (int i = 0; i < mi_bin_len; i++)
            {
                if (i == idx1 || i == idx2 || i == idx3)
                {
                    continue;
                }
                for (int j = 0; j < rot_hist[i].size(); j++)
                {
                    int idx = rot_hist[i][j];
                    if (vn_matches12[idx] >= 0)
                    {
                        vn_matches12[idx] = -1;
                        nmatches--;
                    }
                }
            }
        }
        for (int i = 0; i < vn_matches12.size(); i++)
        {
            if (vn_matches12[i] >= 0)
            {
                vp_prematched[i] = F2.mv_orb_unkeypoints[vn_matches12[i]].pt;
            }
        }
        return nmatches;
    }

    int Matcher::Compute_descriptor_distance(const Mat &descriptor1, const Mat &descriptor2)
    {
        const int *p1 = descriptor1.ptr<int32_t>();
        const int *p2 = descriptor2.ptr<int32_t>();
        int dist = 0;
        for (int i = 0; i < 8; i++, p1++, p2++)
        {
            unsigned int v = *p1 ^ *p2;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }
        return dist;
    }

    void Matcher::Compute_three_maxima(vector<int> *histo, int histo_len, int &idx1, int &idx2, int &idx3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;
        for (int i = 0; i < histo_len; i++)
        {
            const int s = histo[i].size();
            if (s > max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                idx3 = idx2;
                idx2 = idx1;
                idx1 = i;
            }
            else if (s > max2)
            {
                max3 = max2;
                max2 = s;
                idx3 = idx2;
                idx2 = i;
            }
            else if (s > max3)
            {
                max3 = s;
                idx3 = i;
            }
        }
        if (max2 < 0.1f * (float)max1)
        {
            idx2 = -1;
            idx3 = -1;
        }
        else if (max3 < 0.1f * (float)max1)
        {
            idx3 = -1;
        }
    }

    void Matcher::Set_find_match_parameters(int th_low, float best_second_ratio, bool check_orientation, int bin_len)
    {
        mi_th_low = th_low;
        mf_best_secondbest_ratio = best_second_ratio;
        mb_check_orientation = check_orientation;
        mi_bin_len = bin_len;
    }
}