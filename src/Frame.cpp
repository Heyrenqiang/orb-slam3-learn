#include "../include/Frame.hpp"
#include "../include/ORBFeature.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    long unsigned int KeyFrame::sm_curid = 0;
    bool Frame::mb_initial_image_bound = true;
    float Frame::mf_grid_element_width_inv;
    float Frame::mf_grid_element_height_inv;
    float Frame::mf_box_minx;
    float Frame::mf_box_miny;
    float Frame::mf_box_maxx;
    float Frame::mf_box_maxy;
    Frame::Frame() {}
    KeyFrame::KeyFrame()
    {
        mb_bad = false;
        mi_id = sm_curid++;
        mb_first_connect = true;
    }
    void KeyFrame::Init_keyframe()
    {
        mvp_mappoints = vector<MapPoint *>(mi_feature_num, static_cast<MapPoint *>(nullptr));
    }
    Frame::Frame(const Mat &im, ORBFeature *orbfeature, int feature_num_to_extract)
    {
        // m_image = im.clone();
        orbfeature->extract_orb_compute_descriptor(im, feature_num_to_extract, mv_orb_keypoints, mm_descriptors);
        mi_feature_num = mv_orb_keypoints.size();
        cout << "feature num:" << mi_feature_num << endl;
        orbfeature->undistort_keypoints(mv_orb_keypoints, mv_orb_unkeypoints, mi_feature_num);
        if (mb_initial_image_bound)
        {
            orbfeature->compute_image_bounds(im, mf_box_minx, mf_box_miny, mf_box_maxx, mf_box_maxy);
            mf_grid_element_width_inv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mf_box_maxx - mf_box_minx);
            mf_grid_element_height_inv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mf_box_maxy - mf_box_miny);
            mb_initial_image_bound = false;
        }
        assign_features_to_grid();
    }

    void Frame::assign_features_to_grid()
    {
        int cols = FRAME_GRID_COLS;
        int rows = FRAME_GRID_ROWS;
        const int ncells = rows * cols;
        int nreserve = 0.5f * mi_feature_num / ncells;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                mv_grid[j][i].reserve(nreserve);
            }
        }
        for (int i = 0; i < mi_feature_num; i++)
        {
            const KeyPoint &kp = mv_orb_unkeypoints[i];
            int row, col;
            if (Pos_in_grid(kp, row, col))
            {
                mv_grid[col][row].push_back(i);
                // cout<<"grid:"<<mv_grid[col][row].size()<<endl;
            }
        }
    }
    bool Frame::Pos_in_grid(const KeyPoint &kp, int &row, int &col)
    {
        col = cvFloor((kp.pt.x - mf_box_minx) * mf_grid_element_width_inv);
        row = cvFloor((kp.pt.y - mf_box_miny) * mf_grid_element_height_inv);
        if (col < 0 || col >= FRAME_GRID_COLS || row < 0 || row >= FRAME_GRID_ROWS)
        {
            mi_feature_num--;
            return false;
        }
        return true;
    }

    vector<int> Frame::Get_candidate_points_to_match(Point2f &vp_prematched, int window_size, int min_level, int max_level)
    {
        float x = vp_prematched.x;
        float y = vp_prematched.y;
        vector<int> candidate_index;
        candidate_index.reserve(mi_feature_num);
        float factor_x = (float)window_size;
        float factor_y = (float)window_size;
        const int min_xindex = max(0, (int)floor((x - mf_box_minx - factor_x) * mf_grid_element_width_inv));
        if (min_xindex >= FRAME_GRID_COLS)
        {
            return candidate_index;
        }
        const int max_xindex = min((int)FRAME_GRID_COLS - 1, (int)floor((x - mf_box_minx + factor_x) * mf_grid_element_width_inv));
        if (max_xindex < 0)
        {
            return candidate_index;
        }
        const int min_yindex = max(0, (int)floor((y - mf_box_miny - factor_y) * mf_grid_element_height_inv));
        if (min_yindex >= FRAME_GRID_ROWS)
        {
            return candidate_index;
        }
        const int max_yindex = min((int)FRAME_GRID_ROWS - 1, (int)floor((y - mf_box_miny + factor_y) * mf_grid_element_height_inv));
        if (max_yindex < 0)
        {
            return candidate_index;
        }
        for (int ix = min_xindex; ix <= max_xindex; ix++)
        {
            for (int iy = min_yindex; iy <= max_yindex; iy++)
            {
                vector<int> kps_in_cell = mv_grid[ix][iy];
                if (kps_in_cell.empty())
                {
                    continue;
                }
                for (int i = 0; i < kps_in_cell.size(); i++)
                {
                    KeyPoint kp = mv_orb_unkeypoints[kps_in_cell[i]];
                    // TODO different from orign code
                    if (kp.octave < min_level || kp.octave > max_level)
                    {
                        continue;
                    }
                    const float distx = kp.pt.x - x;
                    const float disty = kp.pt.y - y;
                    if (fabs(distx) < factor_x && fabs(disty) < factor_y)
                    {
                        candidate_index.push_back(kps_in_cell[i]);
                    }
                }
            }
        }
        return candidate_index;
    }

    void Frame::Set_pose(Mat Tcw)
    {
        mm_Tcw = Tcw.clone();
        mm_Rcw = mm_Tcw.rowRange(0, 3).colRange(0, 3);
        mm_tcw = mm_Tcw.rowRange(0, 3).col(3);
        mm_Rwc = mm_Rcw.t();
        mm_twc = -mm_Rwc * mm_tcw;
        mm_Twc = Mat::eye(4, 4, CV_32F);
        mm_Rwc.copyTo(mm_Twc.rowRange(0, 3).colRange(0, 3));
        mm_twc.copyTo(mm_Twc.rowRange(0, 3).col(3));
    }

    void KeyFrame::Compute_bow()
    {
    }
    void KeyFrame::Add_mappoint(MapPoint *p_mappoint, int &idx)
    {
        mvp_mappoints[idx] = p_mappoint;
    }
    void KeyFrame::Update_connections()
    {
        map<KeyFrame *, int> KF_counter;
        for (vector<MapPoint *>::iterator vit = mvp_mappoints.begin(); vit != mvp_mappoints.end(); vit++)
        {
            MapPoint *pmp = *vit;
            if (!pmp)
            {
                continue;
            }
            if (pmp->mb_bad)
            {
                continue;
            }
            map<KeyFrame *, int> observations = pmp->mmap_observation;
            for (map<KeyFrame *, int>::iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                if (mit->first->mi_id == this->mi_id || mit->first->mb_bad || mit->first->mp_map != this->mp_map)
                {
                    continue;
                }
                KF_counter[mit->first]++;
            }
        }
        if (KF_counter.empty())
        {
            return;
        }
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        // TODO
        int th = 15;
        vector<pair<int, KeyFrame *>> v_pair;
        v_pair.reserve(KF_counter.size());
        for (map<KeyFrame *, int>::iterator mit = KF_counter.begin(); mit != KF_counter.end(); mit++)
        {
            if (mit->second > nmax)
            {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            if (mit->second > th)
            {
                v_pair.push_back(make_pair(mit->second, mit->first));
                (mit->first)->Add_connection(this, mit->second);
            }
        }
        if (v_pair.empty())
        {
            v_pair.push_back(make_pair(nmax, pKFmax));
            pKFmax->Add_connection(this, nmax);
        }
        sort(v_pair.begin(), v_pair.end());
        list<KeyFrame *> l_KF;
        list<int> l_weights;
        for (int i = 0; i < v_pair.size(); i++)
        {
            l_KF.push_front(v_pair[i].second);
            l_weights.push_front(v_pair[i].first);
        }
        mmap_KF_connection_weight = KF_counter;
        mv_ordered_KF = vector<KeyFrame *>(l_KF.begin(), l_KF.end());
        mv_ordered_weight = vector<int>(l_weights.begin(), l_weights.end());
        if (mb_first_connect && mi_id != mp_map->mi_iniKF_id)
        {
            mp_parent = mv_ordered_KF.front();
            mp_parent->Add_child(this);
            mb_first_connect = false;
        }
    }
    set<MapPoint *> KeyFrame::Get_mappoints()
    {
        set<MapPoint *> s;
        for (int i = 0; i < mvp_mappoints.size(); i++)
        {
            if (!mvp_mappoints[i])
            {
                continue;
            }
            MapPoint *p_mp = mvp_mappoints[i];
            if (!p_mp->Is_bad())
            {
                s.insert(p_mp);
            }
        }
        return s;
    }
    float KeyFrame::Compute_scene_median_depth(int q)
    {
    }
    int KeyFrame::Tracked_mappoints(const int &minobs)
    {
    }
    Mat KeyFrame::Get_pose()
    {
    }
    Mat KeyFrame::Get_poseinv()
    {
    }
    void KeyFrame::Set_pose(Mat &tcw)
    {
    }
    vector<MapPoint *> KeyFrame::Get_vect_mappoints()
    {
    }
    void KeyFrame::Set_map(Map *p_map)
    {
        mp_map = p_map;
    }
    void KeyFrame::Add_connection(KeyFrame *pKF, int weight)
    {
        if (!mmap_KF_connection_weight.count(pKF))
        {
            mmap_KF_connection_weight[pKF] = weight;
        }
        else if (mmap_KF_connection_weight[pKF] != weight)
        {
            mmap_KF_connection_weight[pKF] = weight;
        }
        else
        {
            return;
        }
        Update_best_covisibles();
    }
    void KeyFrame::Add_child(KeyFrame *pKF)
    {
        msp_children.insert(pKF);
    }
    void KeyFrame::Update_best_covisibles()
    {
        vector<pair<int, KeyFrame *>> v_pair;
        v_pair.reserve(mmap_KF_connection_weight.size());
        for (map<KeyFrame *, int>::iterator mit = mmap_KF_connection_weight.begin(); mit != mmap_KF_connection_weight.end(); mit++)
        {
            v_pair.push_back(make_pair(mit->second, mit->first));
        }
        sort(v_pair.begin(), v_pair.end());
        list<KeyFrame *> l_KF;
        list<int> l_weight;
        for (int i = 0; i < v_pair.size(); i++)
        {
            if (!v_pair[i].second->mb_bad)
            {
                l_KF.push_front(v_pair[i].second);
                l_weight.push_front(v_pair[i].first);
            }
        }
        mv_ordered_KF = vector<KeyFrame *>(l_KF.begin(), l_KF.end());
        mv_ordered_weight = vector<int>(l_weight.begin(), l_weight.end());
    }

}
