#include "../include/MultipleViewGeometry.hpp"

namespace ORBSLAM
{
    MultipleViewGeometry::MultipleViewGeometry(/* args */)
    {
    }

    MultipleViewGeometry::~MultipleViewGeometry()
    {
    }

    void MultipleViewGeometry::Set_param(Param *param)
    {
        mp_param = param;
        mm_camera_intrinsics = param->mm_camera_intrinsics.clone();
        mf_sigma = 1.0;
        mf_reproj_th = 2;
    }

    void MultipleViewGeometry::Find_fundamental(vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<Match> &matches12, int &itration_num, vector<vector<int>> &candidates, Mat &F21, vector<bool> &vb_match_inline, int &ninliners)
    {

        vector<Point2f> v_np2f1;
        vector<Point2f> v_np2f2;
        v_np2f1.resize(matches12.size());
        v_np2f2.resize(matches12.size());
        Mat T1, T2, T2t;
        Normalize(vp2f_1, v_np2f1, T1);
        Normalize(vp2f_2, v_np2f2, T2);
        T2t = T2.t();

        vector<Point2f> vp1_forF(8);
        vector<Point2f> vp2_forF(8);
        Mat tmp_F21;

        vb_match_inline = vector<bool>(matches12.size(), false);
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
            cur_score = Check_fundamental(tmp_F21, vp2f_1, vp2f_2, vb_cur_match_inline, ncurinline, mf_sigma);
            if (cur_score > score)
            {
                F21 = tmp_F21.clone();
                vb_match_inline = vb_cur_match_inline;
                ninliners = ncurinline;
                score = cur_score;
            }
        }
        // cout << "num inline:" << ninliners << endl;
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

    float MultipleViewGeometry::Check_fundamental(Mat &F21, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinliner, int &num_inline, float sigma)
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

    bool MultipleViewGeometry::ReconstructF(int ninline, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinline, Mat &F21, Mat &R21, Mat &t21, vector<Point3f> &vp3d, vector<bool> &vb_triangulated, float min_parallax, int min_triangulated)
    {
        Mat E21 = mm_camera_intrinsics.t() * F21 * mm_camera_intrinsics;
        // cout << "E21:" << E21 << endl;
        Mat R1, R2, t;
        DecomposeE(E21, R1, R2, t);
        cout << "R1:" << R1 << endl;
        cout << "R2:" << R2 << endl;
        cout << "t:" << t << endl;
        vector<Point3f> vp3f_1, vp3f_2, vp3f_3, vp3f_4;
        vector<bool> vb_triangulated1, vb_triangulated2, vb_triangulated3, vb_triangulated4;
        float parallax1, parallax2, parallax3, parallax4;
        int ngood1, ngood2, ngood3, ngood4;
        Mat t1, t2;
        t1 = t;
        t2 = -t;
        if (determinant(R1) < 0)
        {
            R1 = -R1;
        }
        if (determinant(R2) < 0)
        {
            R2 = -R2;
        }
        ngood1 = CheckRT(R1, t1, vp2f_1, vp2f_2, vb_ifinline, vp3f_1, vb_triangulated1, parallax1);
        ngood2 = CheckRT(R2, t1, vp2f_1, vp2f_2, vb_ifinline, vp3f_2, vb_triangulated2, parallax2);
        ngood3 = CheckRT(R1, t2, vp2f_1, vp2f_2, vb_ifinline, vp3f_3, vb_triangulated3, parallax3);
        ngood4 = CheckRT(R2, t2, vp2f_1, vp2f_2, vb_ifinline, vp3f_4, vb_triangulated4, parallax4);

        int max_good = max(ngood1, max(ngood2, max(ngood3, ngood4)));

        int min_good = max(static_cast<int>(0.9 * ninline), min_triangulated);
        int nsimilar = 0;
        if (ngood1 > 0.7 * max_good)
        {
            nsimilar++;
        }
        if (ngood2 > 0.7 * max_good)
        {
            nsimilar++;
        }
        if (ngood3 > 0.7 * max_good)
        {
            nsimilar++;
        }
        if (ngood4 > 0.7 * max_good)
        {
            nsimilar++;
        }
        if (max_good < min_good || nsimilar > 1)
        {
            return false;
        }

        if (max_good == ngood1)
        {
            if (parallax1 > min_parallax)
            {
                vp3d = vp3f_1;
                vb_triangulated = vb_triangulated2;
                R1.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
            else
            {
                cout << "small parallax! " <<parallax1<<endl;
            }
        }
        else if (max_good == ngood2)
        {
            if (parallax2 > min_parallax)
            {
                vp3d = vp3f_2;
                vb_triangulated = vb_triangulated2;
                R2.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
            else
            {
                cout << "small parallax! " <<parallax2<< endl;
            }
        }
        else if (max_good == ngood3)
        {
            if (parallax3 > min_parallax)
            {
                vp3d = vp3f_3;
                vb_triangulated = vb_triangulated3;
                R1.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
            else
            {
                cout << "small parallax! " <<parallax3<< endl;
            }
        }
        else if (max_good == ngood4)
        {
            if (parallax4 > min_parallax)
            {
                vp3d = vp3f_4;
                vb_triangulated = vb_triangulated4;
                R2.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
            else
            {
                cout << "small parallax! " <<parallax4<< endl;
            }
        }
        return false;
    }

    void MultipleViewGeometry::DecomposeE(Mat &E21, Mat &R1, Mat &R2, Mat &t)
    {
        Mat u, w, vt;
        SVD::compute(E21, w, u, vt);
        u.col(2).copyTo(t);
        t = -t / norm(t);

        Mat W(3, 3, CV_32F, Scalar(0));
        W.at<float>(0, 1) = -1;
        W.at<float>(1, 0) = 1;
        W.at<float>(2, 2) = 1;

        R1 = u * W * vt;
        R2 = u * W.t() * vt;
    }

    int MultipleViewGeometry::CheckRT(Mat &R, Mat &t, vector<Point2f> &vp2f_1, vector<Point2f> &vp2f_2, vector<bool> &vb_ifinline, vector<Point3f> &vp3f, vector<bool> &vb_triangulated, float &parallax)
    {
        const float fx = mm_camera_intrinsics.at<float>(0, 0);
        const float fy = mm_camera_intrinsics.at<float>(1, 1);
        const float cx = mm_camera_intrinsics.at<float>(0, 2);
        const float cy = mm_camera_intrinsics.at<float>(1, 2);

        vb_triangulated = vector<bool>(vb_ifinline.size(), false);
        vp3f.resize(vb_ifinline.size());
        vector<float> v_cosparallax;
        v_cosparallax.reserve(vb_ifinline.size());

        Mat P1(3, 4, CV_32F, Scalar(0));
        mm_camera_intrinsics.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        Mat O1 = Mat::zeros(3, 1, CV_32F);

        Mat P2(3, 4, CV_32F);
        R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        t.copyTo(P2.rowRange(0, 3).col(3));
        P2 = mm_camera_intrinsics * P2;

        Mat O2 = -R.t() * t;

        int ngood = 0;

        for (int i = 0; i < vb_ifinline.size(); i++)
        {
            if (!vb_ifinline[i])
            {
                continue;
            }
            Mat p3d;
            Triangulate(vp2f_1[i], vp2f_2[i], P1, P2, p3d);
            if (!isfinite(p3d.at<float>(0)) || !isfinite(p3d.at<float>(1)) || !isfinite(p3d.at<float>(2)))
            {
                vb_triangulated[i] = false;
                continue;
            }
            Mat normal1 = p3d - O1;
            float dist1 = norm(normal1);
            Mat normal2 = p3d - O2;
            float dist2 = norm(normal2);

            float cos_parallax = normal1.dot(normal2) / (dist1 * dist2);
            if (p3d.at<float>(2) <= 0 && cos_parallax < 0.99998)
            {
                continue;
            }
            Mat p3d_in2 = R * p3d + t;
            if (p3d_in2.at<float>(2) <= 0 && cos_parallax < 0.99998)
            {
                continue;
            }
            float im1x, im1y;
            float inv_z1 = 1.0 / p3d.at<float>(2);
            im1x = fx * p3d.at<float>(0) * inv_z1 + cx;
            im1y = fy * p3d.at<float>(1) * inv_z1 + cy;
            float square_error1 = (im1x - vp2f_1[i].x) * (im1x - vp2f_1[i].x) + (im1y - vp2f_1[i].y) * (im1y - vp2f_1[i].y);
            if (square_error1 > mf_reproj_th * mf_reproj_th)
            {
                continue;
            }
            float im2x, im2y;
            float inv_z2 = 1.0 / p3d_in2.at<float>(2);
            im2x = fx * p3d_in2.at<float>(0) * inv_z2 + cx;
            im2y = fy * p3d_in2.at<float>(1) * inv_z2 + cy;
            float square_error2 = (im2x - vp2f_2[i].x) * (im2x - vp2f_2[i].x) + (im2y - vp2f_2[i].y) * (im2y - vp2f_2[i].y);
            if (square_error2 > mf_reproj_th * mf_reproj_th)
            {
                continue;
            }
            v_cosparallax.push_back(cos_parallax);
            vp3f[i] = Point3f(p3d.at<float>(0), p3d.at<float>(1), p3d.at<float>(2));
            ngood++;
            if (cos_parallax < 0.99998)
            {
                vb_triangulated[i] = true;
            }
        }
        if (ngood > 0)
        {
            sort(v_cosparallax.begin(), v_cosparallax.end());
            int idx = min(50, int(v_cosparallax.size() - 1));
            parallax = acos(v_cosparallax[idx]) * 180 / CV_PI;
        }
        else
        {
            parallax = 0;
        }
        return ngood;
    }

    void MultipleViewGeometry::Triangulate(const Point2f &p2f1, const Point2f &p2f2, const Mat &P1, const Mat &P2, Mat &p3d)
    {
        Mat A(4, 4, CV_32F);
        A.row(0) = p2f1.x * P1.row(2) - P1.row(0);
        A.row(1) = p2f1.y * P1.row(2) - P1.row(1);
        A.row(2) = p2f2.x * P2.row(2) - P2.row(0);
        A.row(3) = p2f2.y * P2.row(2) - P2.row(1);

        Mat u, w, vt;
        SVD::compute(A, w, u, vt, SVD::MODIFY_A | SVD::FULL_UV);
        p3d = vt.row(3).t();
        p3d = p3d.rowRange(0, 3) / p3d.at<float>(3);
    }
}