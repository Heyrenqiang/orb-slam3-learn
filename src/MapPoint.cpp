#include "../include/MapPoint.hpp"
namespace ORBSLAM
{
    MapPoint::MapPoint(/* args */)
    {
    }

    MapPoint::~MapPoint()
    {
    }

    MapPoint::MapPoint(Mat &pos)
    {
        pos.copyTo(mm_world_pose);
        m_nobs = 0;
        mb_bad = false;
    }

    MapPoint::MapPoint(Mat &pos, KeyFrame *p_KF, Map *p_map)
    {
    }

    void MapPoint::Set_ref_keframe(KeyFrame *p_KF)
    {
        mp_ref_keyframe = p_KF;
    }

    void MapPoint::Set_map(Map *p_map)
    {
        mp_map = p_map;
    }

    void MapPoint::Add_key_frame(KeyFrame *KF)
    {
    }

    void MapPoint::Add_observation(KeyFrame *p_KF, int idx)
    {
        mmap_observation[p_KF] = idx;
        m_nobs++;
    }

    void MapPoint::Compute_distinctive_descriptors(Matcher *p_matcher)
    {
        if (mb_bad)
        {
            return;
        }
        if (mmap_observation.empty())
        {
            return;
        }
        vector<Mat> v_descriptor;
        v_descriptor.reserve(mmap_observation.size());
        for (map<KeyFrame *, int>::iterator mit = mmap_observation.begin(), mend = mmap_observation.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            if (!pKF->mb_bad)
            {
                v_descriptor.push_back(pKF->mm_descriptors.row(mit->second));
            }
        }
        if (v_descriptor.empty())
        {
            return;
        }
        const int n = v_descriptor.size();
        float distance[n][n];
        for (int i = 0; i < n; i++)
        {
            distance[i][i] = 0;
            for (int j = i + 1; j < n; j++)
            {
                int dist_ij = p_matcher->Compute_descriptor_distance(v_descriptor[i], v_descriptor[j]);
                distance[i][j] = dist_ij;
                distance[j][i] = dist_ij;
            }
        }
        int best_median = INT_MAX;
        int bestidx = 0;
        for (int i = 0; i < n; i++)
        {
            vector<int> v_dist(distance[i], distance[i] + n);
            sort(v_dist.begin(), v_dist.end());
            int median = v_dist[(n - 1) / 2];
            if (median < best_median)
            {
                best_median = median;
                bestidx = i;
            }
        }
        mm_descrptor = v_descriptor[bestidx].clone();
    }

    void MapPoint::Update_normal_and_depth()
    {
        if (mb_bad)
        {
            return;
        }
        if (mmap_observation.empty())
        {
            return;
        }
        Mat normal = Mat::zeros(3, 1, CV_32F);
        int n = 0;
        for (map<KeyFrame *, int>::iterator mit = mmap_observation.begin(); mit != mmap_observation.end(); mit++)
        {
            KeyFrame *pKF = mit->first;
            int idx = mit->second;
            Mat normali = mm_world_pose - pKF->mm_twc;
            normal = normal + normali / norm(normali);
            n++;
        }
        Mat PC = mm_world_pose - mp_ref_keyframe->mm_twc;
        const float dist = norm(PC);
        int index = mmap_observation[mp_ref_keyframe];
        int level = mp_ref_keyframe->mv_orb_unkeypoints[index].octave;
        const float level_scale_factor = mp_ref_keyframe->mvd_scale_factor[level];
        const int nlevel = mp_ref_keyframe->mi_nlevel;
        mf_max_distance = dist * level_scale_factor;
        mf_min_distance = mf_max_distance / mp_ref_keyframe->mvd_scale_factor[nlevel - 1];
        mm_normal_vector = normal / n;
    }

    bool MapPoint::Is_bad()
    {
        return false;
    }

    void MapPoint::Set_world_pose(const Mat &pos)
    {
    }

    Mat MapPoint::Get_world_pose()
    {
    }

}