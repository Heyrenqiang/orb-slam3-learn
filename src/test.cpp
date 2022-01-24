#include "iostream"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/ORBFeature.hpp"
#include "../include/Frame.hpp"
#include "../include/DataIO.hpp"
#include "../include/Matcher.hpp"
#include "../include/SlamProcess.hpp"
#include <queue>
#include <ctime>
using namespace std;
using namespace cv;

void test_priority_queue(int num)
{
    priority_queue<int> q;
    int j = 0;
    while (j < num)
    {
        for (int i = 0; i < 100; i++)
        {
            q.push(i);
        }
        while (!q.empty())
        {
            q.pop();
        }
        j++;
    }
}
void test_list(int num)
{
    list<int> l;
    int j = 0;
    while (j < num)
    {
        for (int i = 0; i < 100; i++)
        {
            l.push_back(i);
        }
        list<int>::iterator lit = l.begin();
        while (lit == l.end())
        {
            lit = l.erase(lit);
        }
        j++;
    }
}

int main()
{
    DataIO dataio;
    dataio.Set_image_dir("/home/heyrenqiang/data/MH01/mav0/cam0/data");
    dataio.Set_image_timestamp_dir("/home/heyrenqiang/data/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt");
    vector<string> v_image_names;
    vector<double> v_timestamps;
    dataio.Read_image(v_image_names, v_timestamps);

    // ORBSLAM::ORBFeature *orbfeature_ptr = new ORBSLAM::ORBFeature(1.2, 8);
    // ORBSLAM::Matcher *matcher = new ORBSLAM::Matcher();
    // matcher->Set_find_match_parameters(30, 0.4, true, 180);
    ORBSLAM::SlamProcess *slamprocess = new ORBSLAM::SlamProcess();

    // vector<int> v_initial_matches;
    // vector<Point2f> v_prematched_keypoints;

    // ORBSLAM::Frame initial_frame, current_frame;

    // Mat im1, im2;

    int nimages = v_image_names.size();
    for (int i = 0; i < nimages; i++)
    {
        Mat im;
        im = imread(v_image_names[i], IMREAD_UNCHANGED);
        if (im.channels() == 3)
        {
            cvtColor(im, im, COLOR_RGB2GRAY);
        }
        else if (im.channels() == 4)
        {
            cvtColor(im, im, COLOR_RGB2GRAY);
        }

        // if (i == 0)
        // {
        //     im1 = im.clone();
        // }
        // else if (i == 1)
        // {
        //     im2 = im.clone();
        // }
        slamprocess->Process(im);
        // ORBFeature orbfeature(1.2,8);
        // shared_ptr<ORBSLAM::ORBFeature> orbfeature_ptr = make_shared<ORBSLAM::ORBFeature>(1.2,8);

        // vector<KeyPoint> orb_keypoints;
        // Mat descriptors;

        // orbfeature_ptr->extract_orb_compute_descriptor(im,1000,orb_keypoints,descriptors);
        // if (i == 0)
        // {
        //     initial_frame = ORBSLAM::Frame(im, orbfeature_ptr, 500);
        //     v_prematched_keypoints.resize(initial_frame.mi_feature_num);
        //     for (int i = 0; i < initial_frame.mi_feature_num; i++)
        //     {
        //         v_prematched_keypoints[i] = initial_frame.mv_orb_unkeypoints[i].pt;
        //     }
        // }
        // else
        // {
        //     current_frame = ORBSLAM::Frame(im, orbfeature_ptr, 500);
        //     int nmatchs;
        //     nmatchs = matcher->Search_for_initialization(initial_frame, current_frame, v_prematched_keypoints, v_initial_matches, 100);

        //     // vector<DMatch> m;
        //     // m.reserve(v_initial_matches.size());
        //     // for (int i = 0; i < v_initial_matches.size(); i++)
        //     // {
        //     //     if (v_initial_matches[i] > 0)
        //     //     {
        //     //         m.push_back(DMatch(i, v_initial_matches[i], 5.0));
        //     //     }
        //     // }

        //     // cv::Mat img_goodmatch;
        //     // cv::drawMatches(im1, initial_frame.mv_orb_keypoints, im2, current_frame.mv_orb_keypoints, m, img_goodmatch);

        //     // cv::imshow("good matches", img_goodmatch);
        //     // cv::waitKey(0);

        //     cout << "namtchs:" << nmatchs << endl;
        // }
    }

    // vector<KeyPoint> vToDistributeKeys;
    // vector<KeyPoint> vToDistributeKeys_by_fast_whole_img;
    // Rect rect = Rect(16, 16, 720, 448);
    // orbfeature.Extract_FASTFeature_by_grid(im, 35, vToDistributeKeys,rect);
    // FAST(im.rowRange(16, 16 + 448).colRange(16, 16 + 720), vToDistributeKeys_by_fast_whole_img, 10, true);
    // cout << "whole feature by fast feature extracted:" << vToDistributeKeys_by_fast_whole_img.size() << endl;

    // vector<KeyPoint> result_points_after_octree;
    // result_points_after_octree = orbfeature.Test_DistribiteOctTree(im, vToDistributeKeys, rect, 1086);
    // result_points_after_octree = orbfeature.Test_DistribiteOctTree_by_prior_queue(im, vToDistributeKeys, rect, 1086);

    // compute_pyramid_scale_info(1.2, 8);

    // mvi_feature_num_in_each_pyrimad.resize(8);
    // compute_feature_num_in_pyramids(1000, 1.2, 8);
    // mvm_image_pyramid.resize(8);
    // compute_pyramid(im, 8);
    // mvi_xrange = compute_xrange_by_y(15);

    // int sum_feature_size = 0;
    // vector<Mat> all_level_descriptors(8);
    // vector<vector<KeyPoint>> all_level_keypoints(8);
    // int border = 16;

    // for (int i = 0; i < 8; i++)
    // {
    //     vector<KeyPoint> keypoints_to_distribute;
    //     int width = mvm_image_pyramid[i].cols;
    //     int height = mvm_image_pyramid[i].rows;
    //     Rect rect(border, border, width - 2 * border, height - 2 * border);
    //     orbfeature.Extract_FASTFeature_by_grid(mvm_image_pyramid[i], 35, keypoints_to_distribute, rect);

    //     vector<KeyPoint> result_points_after_octree;

    //     result_points_after_octree = orbfeature.Test_DistribiteOctTree(mvm_image_pyramid[i], keypoints_to_distribute, rect, mvi_feature_num_in_each_pyrimad[i]);
    //     all_level_keypoints[i] = result_points_after_octree;
    //     int feature_size = result_points_after_octree.size();
    //     sum_feature_size += feature_size;
    //     if (feature_size != 0)
    //     {
    //         compute_centroid_orientation(mvm_image_pyramid[i], result_points_after_octree, mvi_xrange, 15);
    //         Mat descriptors = Mat::zeros(feature_size, 32, CV_8U);
    //         compute_descriptors(mvm_image_pyramid[i], result_points_after_octree, descriptors);
    //         all_level_descriptors[i] = descriptors;
    //     }
    // }

    // for (int i = 0; i < 8; i++)
    // {
    //     Mat img = mvm_image_pyramid[i].clone();
    //     cvtColor(img, img, COLOR_GRAY2RGB);
    //     for (vector<KeyPoint>::iterator kit = all_level_keypoints[i].begin(); kit != all_level_keypoints[i].end(); kit++)
    //     {
    //         circle(img, Point(kit->pt.x, kit->pt.y), 2, Scalar(0, 0, 250), 2);
    //     }
    //     imshow("image", img);
    //     waitKey();
    //     destroyWindow("image");
    // }

    // vector<KeyPoint> all_keypoints(sum_feature_size);
    // Mat all_descriptors = Mat::zeros(sum_feature_size, 32, CV_8U);
    // int mono_index = 0;
    // int stero_index = sum_feature_size - 1;
    // for (int i = 0; i < 8; i++)
    // {
    //     vector<KeyPoint> &temp_v_kepoints = all_level_keypoints[i];
    //     int temp_feature_num = (int)temp_v_kepoints.size();
    //     float scale = mvf_scale_factor[i];
    //     int row = 0;
    //     for (vector<KeyPoint>::iterator kit = temp_v_kepoints.begin(); kit != temp_v_kepoints.end(); kit++)
    //     {
    //         kit->octave = i;
    //         if (i != 0)
    //         {
    //             kit->pt *= scale;
    //         }
    //         if (kit->pt.x >= 0 && kit->pt.x <= 1000)
    //         {
    //             all_keypoints.at(stero_index) = (*kit);
    //             all_level_descriptors[i].row(row).copyTo(all_descriptors.row(stero_index));
    //             stero_index--;
    //         }
    //         else
    //         {
    //             all_keypoints.at(mono_index) = (*kit);
    //             all_level_descriptors[i].row(row).copySize(all_descriptors.row(mono_index));
    //             mono_index++;
    //         }
    //         row++;
    //     }
    // }

    // Mat img = mvm_image_pyramid[0].clone();
    // cvtColor(img, img, COLOR_GRAY2RGB);
    // for (vector<KeyPoint>::iterator kit = all_keypoints.begin(); kit != all_keypoints.end(); kit++)
    // {
    //     circle(img, Point(kit->pt.x, kit->pt.y), 2, Scalar(0, 0, 250), 2);
    // }
    // imshow("image", img);
    // waitKey();
    // destroyWindow("image");

    // vector<Mat> my_image;
    // my_image = compute_pyramid_without_copyMakeBorder(im, 8);
    // for(int i=0;i<8;i++){
    //     Mat diff_im(mvm_image_pyramid[i].rows,mvm_image_pyramid[i].cols,mvm_image_pyramid[i].type());
    //     absdiff(mvm_image_pyramid[i],my_image[i],diff_im);
    //     cout<<diff_im<<endl;
    // }

    cout << "end" << endl;

    // vector<int> x_ranges;
    // x_ranges = compute_xrange_by_y(15);
    // for (int i = 0; i < x_ranges.size(); i++)
    // {
    //     cout << x_ranges[i] << endl;
    // }

    // clock_t start, finish;
    // double duration1,duration2;
    // start = clock();
    // test_list(10000);
    // finish = clock();
    // duration1 = (double)(finish - start) / CLOCKS_PER_SEC;
    // cout << "time cost:" << duration1 << endl;

    // start = clock();
    // test_priority_queue(10000);
    // finish = clock();
    // duration2 = (double)(finish - start) / CLOCKS_PER_SEC;
    // cout << "time cost:" << duration2 << endl;
    // cout<<duration2/duration1<<endl;

    // priority_queue<pair<int,int>> q;
    // pair<int,int> a(1,1);
    // pair<int,int> b(3,4);
    // pair<int,int> c(2,2);
    // q.push(a);
    // q.push(b);
    // q.push(c);
    // cout<<q.top().second<<endl;
    // priority_queue<int> q;
    // const int *p;
    // int a = 1;
    // q.push(a);
    // p = &q.top();

    // cout<<q.top()<<endl;

    return 0;
}