#include "../include/Frame.hpp"
#include "../include/ORBFeature.hpp"
using namespace std;
using namespace cv;
namespace ORBSLAM
{
    bool Frame::mb_initial_image_bound = true;
    float Frame::mf_grid_element_width_inv;
    float Frame::mf_grid_element_height_inv;
    float Frame::mf_box_minx;
    float Frame::mf_box_miny;
    float Frame::mf_box_maxx;
    float Frame::mf_box_maxy;
    Frame::Frame(){}
    Frame::Frame(const Mat &im, ORBFeature *orbfeature, int feature_num_to_extract)
    {
        m_image = im.clone();
        orbfeature->extract_orb_compute_descriptor(im, feature_num_to_extract, mv_orb_keypoints, mm_descriptors);
        mi_feature_num = mv_orb_keypoints.size();
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
        int nreserve = 0.5f * mi_feature_num/ncells;
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

    vector<int> Frame::Get_candidate_points_to_match(Point2f &vp_prematched,int window_size,int min_level,int max_level){
        float x = vp_prematched.x;
        float y = vp_prematched.y;
        vector<int> candidate_index;
        candidate_index.reserve(mi_feature_num);
        float factor_x = (float)window_size;
        float factor_y = (float)window_size;
        const int min_xindex = max(0,(int)floor((x-mf_box_minx-factor_x)*mf_grid_element_width_inv));
        if(min_xindex>=FRAME_GRID_COLS){
            return candidate_index; 
        }
        const int max_xindex = min((int)FRAME_GRID_COLS-1,(int)floor((x-mf_box_minx+factor_x)*mf_grid_element_width_inv));
        if(max_xindex<0){
            return candidate_index;
        }
        const int min_yindex = max(0,(int)floor((y-mf_box_miny-factor_y)*mf_grid_element_height_inv));
        if(min_yindex>=FRAME_GRID_ROWS){
            return candidate_index;
        }
        const int max_yindex = min((int)FRAME_GRID_ROWS-1,(int)floor((y-mf_box_miny+factor_y)*mf_grid_element_height_inv));
        if(max_yindex<0){
            return candidate_index;
        }
        for(int ix=min_xindex;ix<=max_xindex;ix++){
            for(int iy=min_yindex;iy<=max_yindex;iy++){
                vector<int> kps_in_cell = mv_grid[ix][iy];
                if(kps_in_cell.empty()){
                    continue;
                }
                for(int i=0;i<kps_in_cell.size();i++){
                    KeyPoint kp = mv_orb_unkeypoints[kps_in_cell[i]];
                    //TODO different from orign code
                    if(kp.octave<min_level||kp.octave>max_level){
                        continue;
                    }
                    const float distx = kp.pt.x-x;
                    const float disty = kp.pt.y-y;
                    if(fabs(distx)<factor_x&&fabs(disty)<factor_y){
                        candidate_index.push_back(kps_in_cell[i]);
                    }
                }
            }
        }
        return candidate_index;
    }

    void Frame::Set_pose(Mat Tcw){
        mm_Tcw = Tcw.clone();
        mm_Rcw = mm_Tcw.rowRange(0,3).colRange(0,3);
        mm_tcw = mm_Tcw.rowRange(0,3).col(3);
        mm_Rwc = mm_Rcw.t();
        mm_twc = -mm_Rwc*mm_tcw;
    }

}
