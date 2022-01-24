#include "../include/MultipleViewGeometry.hpp"

namespace ORBSLAM
{
    MultipleViewGeometry::MultipleViewGeometry(/* args */)
    {
        mf_sigma = 1.0;
    }

    MultipleViewGeometry::~MultipleViewGeometry()
    {
    }

    void MultipleViewGeometry::Find_fundamental(vector<KeyPoint> &v_kps1, vector<KeyPoint> &v_kps2, vector<Match> &matches12, int &itration_num, vector<vector<int>> &candidates, Mat &F21,vector<bool> &vb_match_inline, int &ninliners)
    {
        vector<Point2f> v_p2f1, v_p2f2;
        v_p2f1.resize(matches12.size());
        v_p2f2.resize(matches12.size());
        vector<Point2f> v_np2f1;
        vector<Point2f> v_np2f2;
        v_np2f1.resize(matches12.size());
        v_np2f2.resize(matches12.size());
        for (int i = 0; i < matches12.size(); i++)
        {
            v_p2f1[i] = v_kps1[matches12[i].first].pt;
            v_p2f2[i] = v_kps2[matches12[i].second].pt;
        }
        Mat T1, T2, T2t;
        Normalize(v_p2f1, v_np2f1, T1);
        Normalize(v_p2f2, v_np2f2, T2);
        T2t = T2.t();

        vector<Point2f> vp1_forF(8);
        vector<Point2f> vp2_forF(8);
        Mat tmp_F21;

        vb_match_inline = vector<bool>(matches12.size(),false);
        vector<bool> vb_cur_match_inline;
        float score = 0.0;
        float cur_score;
        int ncurinline;

        for (int i = 0; i < itration_num; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                int idx = candidates[i][j];
                vp1_forF[j] = v_np2f1[idx];
                vp2_forF[j] = v_np2f2[idx];
            }
            Mat F = ComputeF(vp1_forF, vp2_forF);
            tmp_F21 = T2t * F * T1;
            cur_score = Check_fundamental(tmp_F21,v_p2f1,v_p2f2,vb_cur_match_inline,ncurinline,mf_sigma);
            if (cur_score > score)
            {
                F21 = tmp_F21.clone();
                vb_match_inline = vb_cur_match_inline;
                ninliners = ncurinline;
                score = cur_score;
            }
        }
        cout<<"num inline:"<<ninliners<<endl;
    }

    void MultipleViewGeometry::Normalize(vector<Point2f> &v_p2f, vector<Point2f> &v_np2f, Mat &T)
    {
        int n = v_p2f.size();
        float meanx = 0.0;
        float meany = 0.0;
        for (int i = 0; i < n; i++)
        {
            meanx = meanx + v_p2f[i].x;
            meany = meany + v_p2f[i].y;
        }
        meanx = meanx / n;
        meany = meany / n;

        float meandx = 0.0;
        float meandy = 0.0;
        for (int i = 0; i < n; i++)
        {
            v_np2f[i].x = v_p2f[i].x - meanx;
            v_np2f[i].y = v_p2f[i].y - meany;
            meandx = meandx + fabs(v_np2f[i].x);
            meandy = meandy + fabs(v_np2f[i].y);
        }
        meandx = meandx / n;
        meandy = meandy / n;
        float inv_meandx = 1.0 / meandx;
        float inv_meandy = 1.0 / meandy;
        for (int i = 0; i < n; i++)
        {
            v_np2f[i].x = v_np2f[i].x * inv_meandx;
            v_np2f[i].y = v_np2f[i].y * inv_meandy;
        }
        T = Mat::eye(3, 3, CV_32F);
        T.at<float>(0, 0) = inv_meandx;
        T.at<float>(1, 1) = inv_meandy;
        T.at<float>(0, 2) = -meanx * inv_meandx;
        T.at<float>(1, 2) = -meany * inv_meandy;
    }

    Mat MultipleViewGeometry::ComputeF(vector<Point2f> &vp2f1, vector<Point2f> &vp2f2)
    {
        int n = vp2f1.size();
        Mat A(n, 9, CV_32F);
        for (int i = 0; i < n; i++)
        {
            const float u1 = vp2f1[i].x;
            const float v1 = vp2f1[i].y;
            const float u2 = vp2f2[i].x;
            const float v2 = vp2f2[i].y;

            A.at<float>(i, 0) = u2 * u1;
            A.at<float>(i, 1) = u2 * v1;
            A.at<float>(i, 2) = u2;
            A.at<float>(i, 3) = v2 * u1;
            A.at<float>(i, 4) = v2 * v1;
            A.at<float>(i, 5) = v2;
            A.at<float>(i, 6) = u1;
            A.at<float>(i, 7) = v1;
            A.at<float>(i, 8) = 1;
        }
        Mat u, w, vt;
        SVDecomp(A, w, u, vt, SVD::MODIFY_A | SVD::FULL_UV);
        Mat Fpre = vt.row(8).reshape(0, 3);
        SVDecomp(Fpre, w, u, vt, SVD::MODIFY_A | SVD::FULL_UV);
        w.at<float>(2) = 0;
        return u * Mat::diag(w) * vt;
    }

    float MultipleViewGeometry::Check_fundamental(Mat &F21, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> vb_ifinliner, int &num_inline, float sigma)
    {
        const int n = vp2f_1.size();
        const float f11 = F21.at<float>(0, 0);
        const float f12 = F21.at<float>(0, 1);
        const float f13 = F21.at<float>(0, 2);
        const float f21 = F21.at<float>(1, 0);
        const float f22 = F21.at<float>(1, 1);
        const float f23 = F21.at<float>(1, 2);
        const float f31 = F21.at<float>(2, 0);
        const float f32 = F21.at<float>(2, 1);
        const float f33 = F21.at<float>(2, 2);
        vb_ifinliner.resize(n);
        float score = 0.0;
        num_inline = 0;
        const float th = 3.841;
        const float inv_sigma2 = 1.0 / (sigma * sigma);
        for (int i = 0; i < n; i++)
        {
            bool b_in = true;
            const float u1 = vp2f_1[i].x;
            const float v1 = vp2f_1[i].y;
            const float u2 = vp2f_2[i].x;
            const float v2 = vp2f_2[i].y;

            const float a2 = f11 * u1 + f12 * v1 + f13;
            const float b2 = f21 * u1 + f22 * v1 + f23;
            const float c2 = f31 * u1 + f32 * v1 + f33;
            const float num2 = a2 * u2 + b2 * v2 + c2;
            const float dist_square2 = num2 * num2 / (a2 * a2 + b2 * b2);
            const float chi_square2 = dist_square2 * inv_sigma2;
            if (chi_square2 > th)
            {
                b_in = false;
            }
            else
            {
                score += th - chi_square2;
            }
            const float a1 = f11 * u2 + f21 * v2 + f31;
            const float b1 = f12 * u2 + f22 * v2 + f32;
            const float c1 = f13 * u2 + f23 * v2 + f33;
            const float num1 = a1 * u1 + b1 * v1 + c1;
            const float dist_square1 = num1 * num1 / (a1 * a1 + b1 * b1);
            const float chi_square1 = dist_square1 * inv_sigma2;
            if (chi_square1 > th)
            {
                b_in = false;
            }
            else
            {
                score += th - chi_square1;
            }
            if (b_in)
            {
                vb_ifinliner[i] = true;
                num_inline++;
            }
            else
            {
                vb_ifinliner[i] = false;
            }
        }
        return score;
    }
}