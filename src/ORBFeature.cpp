#include "../include/ORBFeature.hpp"

using namespace std;
using namespace cv;
// #define _DEBUG

static int bit_pattern_31_[256 * 4] =
    {
        8, -3, 9, 5 /*mean (0), correlation (0)*/,
        4, 2, 7, -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
        -11, 9, -8, 2 /*mean (3.37382e-05), correlation (0.0617409)*/,
        7, -12, 12, -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
        2, -13, 2, 12 /*mean (0.000134953), correlation (0.085099)*/,
        1, -7, 1, 6 /*mean (0.000528565), correlation (0.0857175)*/,
        -2, -10, -2, -4 /*mean (0.0188821), correlation (0.0985774)*/,
        -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
        -13, -3, -12, -9 /*mean (0.121806), correlation (0.099849)*/,
        10, 4, 11, 9 /*mean (0.122065), correlation (0.093285)*/,
        -13, -8, -8, -9 /*mean (0.162787), correlation (0.0942748)*/,
        -11, 7, -9, 12 /*mean (0.21561), correlation (0.0974438)*/,
        7, 7, 12, 6 /*mean (0.160583), correlation (0.130064)*/,
        -4, -5, -3, 0 /*mean (0.228171), correlation (0.132998)*/,
        -13, 2, -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
        -9, 0, -7, 5 /*mean (0.198234), correlation (0.143636)*/,
        12, -6, 12, -1 /*mean (0.0676226), correlation (0.16689)*/,
        -3, 6, -2, 12 /*mean (0.166847), correlation (0.171682)*/,
        -6, -13, -4, -8 /*mean (0.101215), correlation (0.179716)*/,
        11, -13, 12, -8 /*mean (0.200641), correlation (0.192279)*/,
        4, 7, 5, 1 /*mean (0.205106), correlation (0.186848)*/,
        5, -3, 10, -3 /*mean (0.234908), correlation (0.192319)*/,
        3, -7, 6, 12 /*mean (0.0709964), correlation (0.210872)*/,
        -8, -7, -6, -2 /*mean (0.0939834), correlation (0.212589)*/,
        -2, 11, -1, -10 /*mean (0.127778), correlation (0.20866)*/,
        -13, 12, -8, 10 /*mean (0.14783), correlation (0.206356)*/,
        -7, 3, -5, -3 /*mean (0.182141), correlation (0.198942)*/,
        -4, 2, -3, 7 /*mean (0.188237), correlation (0.21384)*/,
        -10, -12, -6, 11 /*mean (0.14865), correlation (0.23571)*/,
        5, -12, 6, -7 /*mean (0.222312), correlation (0.23324)*/,
        5, -6, 7, -1 /*mean (0.229082), correlation (0.23389)*/,
        1, 0, 4, -5 /*mean (0.241577), correlation (0.215286)*/,
        9, 11, 11, -13 /*mean (0.00338507), correlation (0.251373)*/,
        4, 7, 4, 12 /*mean (0.131005), correlation (0.257622)*/,
        2, -1, 4, 4 /*mean (0.152755), correlation (0.255205)*/,
        -4, -12, -2, 7 /*mean (0.182771), correlation (0.244867)*/,
        -8, -5, -7, -10 /*mean (0.186898), correlation (0.23901)*/,
        4, 11, 9, 12 /*mean (0.226226), correlation (0.258255)*/,
        0, -8, 1, -13 /*mean (0.0897886), correlation (0.274827)*/,
        -13, -2, -8, 2 /*mean (0.148774), correlation (0.28065)*/,
        -3, -2, -2, 3 /*mean (0.153048), correlation (0.283063)*/,
        -6, 9, -4, -9 /*mean (0.169523), correlation (0.278248)*/,
        8, 12, 10, 7 /*mean (0.225337), correlation (0.282851)*/,
        0, 9, 1, 3 /*mean (0.226687), correlation (0.278734)*/,
        7, -5, 11, -10 /*mean (0.00693882), correlation (0.305161)*/,
        -13, -6, -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
        10, 7, 12, 1 /*mean (0.125517), correlation (0.31089)*/,
        -6, -3, -6, 12 /*mean (0.131748), correlation (0.312779)*/,
        10, -9, 12, -4 /*mean (0.144827), correlation (0.292797)*/,
        -13, 8, -8, -12 /*mean (0.149202), correlation (0.308918)*/,
        -13, 0, -8, -4 /*mean (0.160909), correlation (0.310013)*/,
        3, 3, 7, 8 /*mean (0.177755), correlation (0.309394)*/,
        5, 7, 10, -7 /*mean (0.212337), correlation (0.310315)*/,
        -1, 7, 1, -12 /*mean (0.214429), correlation (0.311933)*/,
        3, -10, 5, 6 /*mean (0.235807), correlation (0.313104)*/,
        2, -4, 3, -10 /*mean (0.00494827), correlation (0.344948)*/,
        -13, 0, -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
        -13, -7, -12, 12 /*mean (0.103385), correlation (0.342715)*/,
        -13, 3, -11, 8 /*mean (0.134222), correlation (0.322922)*/,
        -7, 12, -4, 7 /*mean (0.153284), correlation (0.337061)*/,
        6, -10, 12, 8 /*mean (0.154881), correlation (0.329257)*/,
        -9, -1, -7, -6 /*mean (0.200967), correlation (0.33312)*/,
        -2, -5, 0, 12 /*mean (0.201518), correlation (0.340635)*/,
        -12, 5, -7, 5 /*mean (0.207805), correlation (0.335631)*/,
        3, -10, 8, -13 /*mean (0.224438), correlation (0.34504)*/,
        -7, -7, -4, 5 /*mean (0.239361), correlation (0.338053)*/,
        -3, -2, -1, -7 /*mean (0.240744), correlation (0.344322)*/,
        2, 9, 5, -11 /*mean (0.242949), correlation (0.34145)*/,
        -11, -13, -5, -13 /*mean (0.244028), correlation (0.336861)*/,
        -1, 6, 0, -1 /*mean (0.247571), correlation (0.343684)*/,
        5, -3, 5, 2 /*mean (0.000697256), correlation (0.357265)*/,
        -4, -13, -4, 12 /*mean (0.00213675), correlation (0.373827)*/,
        -9, -6, -9, 6 /*mean (0.0126856), correlation (0.373938)*/,
        -12, -10, -8, -4 /*mean (0.0152497), correlation (0.364237)*/,
        10, 2, 12, -3 /*mean (0.0299933), correlation (0.345292)*/,
        7, 12, 12, 12 /*mean (0.0307242), correlation (0.366299)*/,
        -7, -13, -6, 5 /*mean (0.0534975), correlation (0.368357)*/,
        -4, 9, -3, 4 /*mean (0.099865), correlation (0.372276)*/,
        7, -1, 12, 2 /*mean (0.117083), correlation (0.364529)*/,
        -7, 6, -5, 1 /*mean (0.126125), correlation (0.369606)*/,
        -13, 11, -12, 5 /*mean (0.130364), correlation (0.358502)*/,
        -3, 7, -2, -6 /*mean (0.131691), correlation (0.375531)*/,
        7, -8, 12, -7 /*mean (0.160166), correlation (0.379508)*/,
        -13, -7, -11, -12 /*mean (0.167848), correlation (0.353343)*/,
        1, -3, 12, 12 /*mean (0.183378), correlation (0.371916)*/,
        2, -6, 3, 0 /*mean (0.228711), correlation (0.371761)*/,
        -4, 3, -2, -13 /*mean (0.247211), correlation (0.364063)*/,
        -1, -13, 1, 9 /*mean (0.249325), correlation (0.378139)*/,
        7, 1, 8, -6 /*mean (0.000652272), correlation (0.411682)*/,
        1, -1, 3, 12 /*mean (0.00248538), correlation (0.392988)*/,
        9, 1, 12, 6 /*mean (0.0206815), correlation (0.386106)*/,
        -1, -9, -1, 3 /*mean (0.0364485), correlation (0.410752)*/,
        -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
        7, 7, 10, 12 /*mean (0.0424202), correlation (0.405663)*/,
        12, -5, 12, 9 /*mean (0.0942645), correlation (0.410422)*/,
        6, 3, 7, 11 /*mean (0.1074), correlation (0.413224)*/,
        5, -13, 6, 10 /*mean (0.109256), correlation (0.408646)*/,
        2, -12, 2, 3 /*mean (0.131691), correlation (0.416076)*/,
        3, 8, 4, -6 /*mean (0.165081), correlation (0.417569)*/,
        2, 6, 12, -13 /*mean (0.171874), correlation (0.408471)*/,
        9, -12, 10, 3 /*mean (0.175146), correlation (0.41296)*/,
        -8, 4, -7, 9 /*mean (0.183682), correlation (0.402956)*/,
        -11, 12, -4, -6 /*mean (0.184672), correlation (0.416125)*/,
        1, 12, 2, -8 /*mean (0.191487), correlation (0.386696)*/,
        6, -9, 7, -4 /*mean (0.192668), correlation (0.394771)*/,
        2, 3, 3, -2 /*mean (0.200157), correlation (0.408303)*/,
        6, 3, 11, 0 /*mean (0.204588), correlation (0.411762)*/,
        3, -3, 8, -8 /*mean (0.205904), correlation (0.416294)*/,
        7, 8, 9, 3 /*mean (0.213237), correlation (0.409306)*/,
        -11, -5, -6, -4 /*mean (0.243444), correlation (0.395069)*/,
        -10, 11, -5, 10 /*mean (0.247672), correlation (0.413392)*/,
        -5, -8, -3, 12 /*mean (0.24774), correlation (0.411416)*/,
        -10, 5, -9, 0 /*mean (0.00213675), correlation (0.454003)*/,
        8, -1, 12, -6 /*mean (0.0293635), correlation (0.455368)*/,
        4, -6, 6, -11 /*mean (0.0404971), correlation (0.457393)*/,
        -10, 12, -8, 7 /*mean (0.0481107), correlation (0.448364)*/,
        4, -2, 6, 7 /*mean (0.050641), correlation (0.455019)*/,
        -2, 0, -2, 12 /*mean (0.0525978), correlation (0.44338)*/,
        -5, -8, -5, 2 /*mean (0.0629667), correlation (0.457096)*/,
        7, -6, 10, 12 /*mean (0.0653846), correlation (0.445623)*/,
        -9, -13, -8, -8 /*mean (0.0858749), correlation (0.449789)*/,
        -5, -13, -5, -2 /*mean (0.122402), correlation (0.450201)*/,
        8, -8, 9, -13 /*mean (0.125416), correlation (0.453224)*/,
        -9, -11, -9, 0 /*mean (0.130128), correlation (0.458724)*/,
        1, -8, 1, -2 /*mean (0.132467), correlation (0.440133)*/,
        7, -4, 9, 1 /*mean (0.132692), correlation (0.454)*/,
        -2, 1, -1, -4 /*mean (0.135695), correlation (0.455739)*/,
        11, -6, 12, -11 /*mean (0.142904), correlation (0.446114)*/,
        -12, -9, -6, 4 /*mean (0.146165), correlation (0.451473)*/,
        3, 7, 7, 12 /*mean (0.147627), correlation (0.456643)*/,
        5, 5, 10, 8 /*mean (0.152901), correlation (0.455036)*/,
        0, -4, 2, 8 /*mean (0.167083), correlation (0.459315)*/,
        -9, 12, -5, -13 /*mean (0.173234), correlation (0.454706)*/,
        0, 7, 2, 12 /*mean (0.18312), correlation (0.433855)*/,
        -1, 2, 1, 7 /*mean (0.185504), correlation (0.443838)*/,
        5, 11, 7, -9 /*mean (0.185706), correlation (0.451123)*/,
        3, 5, 6, -8 /*mean (0.188968), correlation (0.455808)*/,
        -13, -4, -8, 9 /*mean (0.191667), correlation (0.459128)*/,
        -5, 9, -3, -3 /*mean (0.193196), correlation (0.458364)*/,
        -4, -7, -3, -12 /*mean (0.196536), correlation (0.455782)*/,
        6, 5, 8, 0 /*mean (0.1972), correlation (0.450481)*/,
        -7, 6, -6, 12 /*mean (0.199438), correlation (0.458156)*/,
        -13, 6, -5, -2 /*mean (0.211224), correlation (0.449548)*/,
        1, -10, 3, 10 /*mean (0.211718), correlation (0.440606)*/,
        4, 1, 8, -4 /*mean (0.213034), correlation (0.443177)*/,
        -2, -2, 2, -13 /*mean (0.234334), correlation (0.455304)*/,
        2, -12, 12, 12 /*mean (0.235684), correlation (0.443436)*/,
        -2, -13, 0, -6 /*mean (0.237674), correlation (0.452525)*/,
        4, 1, 9, 3 /*mean (0.23962), correlation (0.444824)*/,
        -6, -10, -3, -5 /*mean (0.248459), correlation (0.439621)*/,
        -3, -13, -1, 1 /*mean (0.249505), correlation (0.456666)*/,
        7, 5, 12, -11 /*mean (0.00119208), correlation (0.495466)*/,
        4, -2, 5, -7 /*mean (0.00372245), correlation (0.484214)*/,
        -13, 9, -9, -5 /*mean (0.00741116), correlation (0.499854)*/,
        7, 1, 8, 6 /*mean (0.0208952), correlation (0.499773)*/,
        7, -8, 7, 6 /*mean (0.0220085), correlation (0.501609)*/,
        -7, -4, -7, 1 /*mean (0.0233806), correlation (0.496568)*/,
        -8, 11, -7, -8 /*mean (0.0236505), correlation (0.489719)*/,
        -13, 6, -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
        2, 4, 3, 9 /*mean (0.0323324), correlation (0.501938)*/,
        10, -5, 12, 3 /*mean (0.0399235), correlation (0.494029)*/,
        -6, -5, -6, 7 /*mean (0.0420153), correlation (0.486579)*/,
        8, -3, 9, -8 /*mean (0.0548021), correlation (0.484237)*/,
        2, -12, 2, 8 /*mean (0.0616622), correlation (0.496642)*/,
        -11, -2, -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
        -12, -13, -7, -9 /*mean (0.0829622), correlation (0.495491)*/,
        -11, 0, -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
        5, -3, 11, 8 /*mean (0.0929937), correlation (0.502315)*/,
        -2, -13, -1, 12 /*mean (0.113327), correlation (0.48941)*/,
        -1, -8, 0, 9 /*mean (0.132119), correlation (0.467268)*/,
        -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
        -10, -2, -10, 11 /*mean (0.142173), correlation (0.498714)*/,
        -3, 9, -2, -13 /*mean (0.144141), correlation (0.491973)*/,
        2, -3, 3, 2 /*mean (0.14892), correlation (0.500782)*/,
        -9, -13, -4, 0 /*mean (0.150371), correlation (0.498211)*/,
        -4, 6, -3, -10 /*mean (0.152159), correlation (0.495547)*/,
        -4, 12, -2, -7 /*mean (0.156152), correlation (0.496925)*/,
        -6, -11, -4, 9 /*mean (0.15749), correlation (0.499222)*/,
        6, -3, 6, 11 /*mean (0.159211), correlation (0.503821)*/,
        -13, 11, -5, 5 /*mean (0.162427), correlation (0.501907)*/,
        11, 11, 12, 6 /*mean (0.16652), correlation (0.497632)*/,
        7, -5, 12, -2 /*mean (0.169141), correlation (0.484474)*/,
        -1, 12, 0, 7 /*mean (0.169456), correlation (0.495339)*/,
        -4, -8, -3, -2 /*mean (0.171457), correlation (0.487251)*/,
        -7, 1, -6, 7 /*mean (0.175), correlation (0.500024)*/,
        -13, -12, -8, -13 /*mean (0.175866), correlation (0.497523)*/,
        -7, -2, -6, -8 /*mean (0.178273), correlation (0.501854)*/,
        -8, 5, -6, -9 /*mean (0.181107), correlation (0.494888)*/,
        -5, -1, -4, 5 /*mean (0.190227), correlation (0.482557)*/,
        -13, 7, -8, 10 /*mean (0.196739), correlation (0.496503)*/,
        1, 5, 5, -13 /*mean (0.19973), correlation (0.499759)*/,
        1, 0, 10, -13 /*mean (0.204465), correlation (0.49873)*/,
        9, 12, 10, -1 /*mean (0.209334), correlation (0.49063)*/,
        5, -8, 10, -9 /*mean (0.211134), correlation (0.503011)*/,
        -1, 11, 1, -13 /*mean (0.212), correlation (0.499414)*/,
        -9, -3, -6, 2 /*mean (0.212168), correlation (0.480739)*/,
        -1, -10, 1, 12 /*mean (0.212731), correlation (0.502523)*/,
        -13, 1, -8, -10 /*mean (0.21327), correlation (0.489786)*/,
        8, -11, 10, -6 /*mean (0.214159), correlation (0.488246)*/,
        2, -13, 3, -6 /*mean (0.216993), correlation (0.50287)*/,
        7, -13, 12, -9 /*mean (0.223639), correlation (0.470502)*/,
        -10, -10, -5, -7 /*mean (0.224089), correlation (0.500852)*/,
        -10, -8, -8, -13 /*mean (0.228666), correlation (0.502629)*/,
        4, -6, 8, 5 /*mean (0.22906), correlation (0.498305)*/,
        3, 12, 8, -13 /*mean (0.233378), correlation (0.503825)*/,
        -4, 2, -3, -3 /*mean (0.234323), correlation (0.476692)*/,
        5, -13, 10, -12 /*mean (0.236392), correlation (0.475462)*/,
        4, -13, 5, -1 /*mean (0.236842), correlation (0.504132)*/,
        -9, 9, -4, 3 /*mean (0.236977), correlation (0.497739)*/,
        0, 3, 3, -9 /*mean (0.24314), correlation (0.499398)*/,
        -12, 1, -6, 1 /*mean (0.243297), correlation (0.489447)*/,
        3, 2, 4, -8 /*mean (0.00155196), correlation (0.553496)*/,
        -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
        8, -13, 12, 12 /*mean (0.0034413), correlation (0.544361)*/,
        -8, -12, -6, -5 /*mean (0.003565), correlation (0.551225)*/,
        2, 2, 3, 7 /*mean (0.00835583), correlation (0.55285)*/,
        10, 6, 11, -8 /*mean (0.00885065), correlation (0.540913)*/,
        6, 8, 8, -12 /*mean (0.0101552), correlation (0.551085)*/,
        -7, 10, -6, 5 /*mean (0.0102227), correlation (0.533635)*/,
        -3, -9, -3, 9 /*mean (0.0110211), correlation (0.543121)*/,
        -1, -13, -1, 5 /*mean (0.0113473), correlation (0.550173)*/,
        -3, -7, -3, 4 /*mean (0.0140913), correlation (0.554774)*/,
        -8, -2, -8, 3 /*mean (0.017049), correlation (0.55461)*/,
        4, 2, 12, 12 /*mean (0.01778), correlation (0.546921)*/,
        2, -5, 3, 11 /*mean (0.0224022), correlation (0.549667)*/,
        6, -9, 11, -13 /*mean (0.029161), correlation (0.546295)*/,
        3, -1, 7, 12 /*mean (0.0303081), correlation (0.548599)*/,
        11, -1, 12, 4 /*mean (0.0355151), correlation (0.523943)*/,
        -3, 0, -3, 6 /*mean (0.0417904), correlation (0.543395)*/,
        4, -11, 4, 12 /*mean (0.0487292), correlation (0.542818)*/,
        2, -4, 2, 1 /*mean (0.0575124), correlation (0.554888)*/,
        -10, -6, -8, 1 /*mean (0.0594242), correlation (0.544026)*/,
        -13, 7, -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
        -13, 12, -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
        6, 0, 11, -13 /*mean (0.065126), correlation (0.552006)*/,
        0, -1, 1, 4 /*mean (0.074224), correlation (0.546372)*/,
        -13, 3, -9, -2 /*mean (0.0808592), correlation (0.554875)*/,
        -9, 8, -6, -3 /*mean (0.0883378), correlation (0.551178)*/,
        -13, -6, -8, -2 /*mean (0.0901035), correlation (0.548446)*/,
        5, -9, 8, 10 /*mean (0.0949843), correlation (0.554694)*/,
        2, 7, 3, -9 /*mean (0.0994152), correlation (0.550979)*/,
        -1, -6, -1, -1 /*mean (0.10045), correlation (0.552714)*/,
        9, 5, 11, -2 /*mean (0.100686), correlation (0.552594)*/,
        11, -3, 12, -8 /*mean (0.101091), correlation (0.532394)*/,
        3, 0, 3, 5 /*mean (0.101147), correlation (0.525576)*/,
        -1, 4, 0, 10 /*mean (0.105263), correlation (0.531498)*/,
        3, -6, 4, 5 /*mean (0.110785), correlation (0.540491)*/,
        -13, 0, -10, 5 /*mean (0.112798), correlation (0.536582)*/,
        5, 8, 12, 11 /*mean (0.114181), correlation (0.555793)*/,
        8, 9, 9, -6 /*mean (0.117431), correlation (0.553763)*/,
        7, -4, 8, -12 /*mean (0.118522), correlation (0.553452)*/,
        -10, 4, -10, 9 /*mean (0.12094), correlation (0.554785)*/,
        7, 3, 12, 4 /*mean (0.122582), correlation (0.555825)*/,
        9, -7, 10, -2 /*mean (0.124978), correlation (0.549846)*/,
        7, 0, 12, -2 /*mean (0.127002), correlation (0.537452)*/,
        -1, -6, 0, -11 /*mean (0.127148), correlation (0.547401)*/};

namespace ORBSLAM
{
    ORBFeature::ORBFeature(double scale, int nlevel)
    {
        compute_pyramid_scale_info(scale, nlevel);
        mvi_xrange = compute_xrange_by_y(15);
        mi_nlevel = nlevel;
        md_scale_factor = scale;

        mm_camera_intrinsics = Mat::eye(3, 3, CV_32F);

        float fx = 458.654;
        float fy = 457.296;
        float cx = 367.215;
        float cy = 248.375;

        mm_camera_intrinsics.at<float>(0, 0) = fx;
        mm_camera_intrinsics.at<float>(0, 2) = cx;
        mm_camera_intrinsics.at<float>(1, 1) = fy;
        mm_camera_intrinsics.at<float>(1, 2) = cy;

        float k1 = -0.28340811;
        float k2 = 0.07395907;
        float p1 = 0.00019359;
        float p2 = 1.76187114e-05;

        mm_distcoef = Mat::zeros(4, 1, CV_32F);
        mm_distcoef.at<float>(0) = k1;
        mm_distcoef.at<float>(1) = k2;
        mm_distcoef.at<float>(2) = p1;
        mm_distcoef.at<float>(3) = p2;
    }

    vector<int> ORBFeature::compute_xrange_by_y(int radius)
    {
        vector<int> xrange;
        xrange.resize(radius + 1);

        const double radius2 = radius * radius;
        int m_by = cvFloor(radius * sqrt(2.0) / 2);

        for (int i = 0, j = radius; i <= m_by; i++)
        {
            xrange[i] = cvRound(sqrt(radius2 - i * i));
            if (i > 0)
            {
                if (xrange[i] != xrange[i - 1])
                {
                    xrange[j] = i - 1;
                    j--;
                }
            }
            if (i == m_by)
            {
                xrange[j] = i;
            }
        }
        return xrange;
    }

    void ORBFeature::compute_pyramid_scale_info(double scale_factor, int nlevel)
    {
        mvd_scale_factor.resize(nlevel);
        mvd_inv_scale_factor.resize(nlevel);
        mvd_level_sigma2.resize(nlevel);
        mvd_inv_level_sigma2.resize(nlevel);
        mvd_scale_factor[0] = 1.0f;
        mvd_inv_scale_factor[0] = 1.0f;
        mvd_level_sigma2[0] = 1.0f;
        mvd_inv_level_sigma2[0] = 1.0f;
        for (int i = 1; i < nlevel; i++)
        {
            mvd_scale_factor[i] = mvd_scale_factor[i - 1] * scale_factor;
            mvd_inv_scale_factor[i] = mvd_inv_scale_factor[i - 1] / scale_factor;
            mvd_level_sigma2[i] = mvd_scale_factor[i] * scale_factor;
            mvd_inv_level_sigma2[i] = mvd_inv_scale_factor[i] / scale_factor;
        }
    }

    void MyExtractorNode::DivideNode(MyExtractorNode &n1, MyExtractorNode &n2, MyExtractorNode &n3, MyExtractorNode &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = ceil(static_cast<float>(BL.y - UL.y) / 2);
        Point2i um(UL.x + halfX, UL.y);
        Point2i ml(UL.x, UL.y + halfY);
        Point2i mm(UL.x + halfX, UL.y + halfY);
        Point2i mr(UR.x, UR.y + halfY);
        Point2i bm(BL.x + halfX, BL.y);

        n1.UL = UL;
        n1.UR = um;
        n1.BL = ml;
        n1.BR = mm;

        n2.UL = um;
        n2.UR = UR;
        n2.BL = mm;
        n2.BR = mr;

        n3.UL = ml;
        n3.UR = mm;
        n3.BL = BL;
        n3.BR = bm;

        n4.UL = mm;
        n4.UR = mr;
        n4.BL = bm;
        n4.BR = BR;

        n1.vKeys.reserve(vKeys.size());
        n2.vKeys.reserve(vKeys.size());
        n3.vKeys.reserve(vKeys.size());
        n4.vKeys.reserve(vKeys.size());

        for (size_t i = 0; i < vKeys.size(); i++)
        {
            const KeyPoint &kp = vKeys[i];
            if (kp.pt.x < mm.x)
            {
                if (kp.pt.y <= mm.y)
                {
                    n1.vKeys.push_back(kp);
                    n1.feature_size++;
                }
                else
                {
                    n3.vKeys.push_back(kp);
                    n3.feature_size++;
                }
            }
            else if (kp.pt.x == mm.x)
            {
                if (kp.pt.y < mm.y)
                {
                    n2.vKeys.push_back(kp);
                    n2.feature_size++;
                }
                else
                {
                    n3.vKeys.push_back(kp);
                    n3.feature_size++;
                }
            }
            else
            {
                if (kp.pt.y < mm.y)
                {
                    n2.vKeys.push_back(kp);
                    n2.feature_size++;
                }
                else
                {
                    n4.vKeys.push_back(kp);
                    n4.feature_size++;
                }
            }
        }
        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }

    void ORBFeature::compute_orb_descriptor(const KeyPoint &kpt, const Mat &im, uchar *desc, int *pattern_array)
    {
        float angle = (float)kpt.angle * factorPI;
        float a = (float)cos(angle), b = (float)sin(angle);
        const uchar *center = &im.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        const int step = (int)im.step;
        int *pattern = pattern_array;
#define GET_VALUE(idx) center[cvRound(pattern[2 * idx] * b + pattern[2 * idx + 1] * a) * step + cvRound(pattern[2 * idx] * a - pattern[2 * idx + 1] * b)]
        for (int i = 0; i < 32; ++i, pattern += 32)
        {
            int t0,  //参与比较的一个特征点的灰度值
                t1,  //参与比较的另一个特征点的灰度值
                val; //描述子这个字节的比较结果

            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1; //描述子本字节的bit0
            t0 = GET_VALUE(2);
            t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1; //描述子本字节的bit1
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2; //描述子本字节的bit2
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3; //描述子本字节的bit3
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4; //描述子本字节的bit4
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5; //描述子本字节的bit5
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6; //描述子本字节的bit6
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7; //描述子本字节的bit7

            //保存当前比较的出来的描述子的这个字节
            desc[i] = (uchar)val;
        } //通过对随机点像素灰度的比较，得出BRIEF描述子，一共是32*8=256位
#undef GET_VALUE
    }

    void ORBFeature::compute_descriptors(const Mat &im, vector<KeyPoint> &kepoints, Mat &descriptors)
    {
        // TODO need clone?
        Mat img = im.clone();
        // cout<<img.rowRange(273-15,273+15).colRange(37-15,37+15)<<endl<<endl;
        GaussianBlur(img, img, Size(7, 7), 2, 2, BORDER_REFLECT_101); // TODO gaussuan size can be changed?
        // descriptors = Mat::zeros((int)kepoints.size(), 32, CV_8UC1);
        for (int i = 0; i < kepoints.size(); i++)
        {
            // if (i == 1616)
            // {
            //     cout<<img.rowRange(273-15,273+15).colRange(37-15,37+15)<<endl;
            //     cout << ".........................................." << endl;
            // }
            compute_orb_descriptor(kepoints[i], img, descriptors.ptr((int)i), bit_pattern_31_);
        }
    }

    void ORBFeature::compute_centroid_orientation(const Mat &im, vector<KeyPoint> &all_keypoints, vector<int> &xrange, int radius)
    {
        for (vector<KeyPoint>::iterator kit = all_keypoints.begin(); kit != all_keypoints.end(); kit++)
        {
            int m_x = 0, m_y = 0;
            const uchar *center = &im.at<uchar>(cvRound(kit->pt.y), cvRound(kit->pt.x));
            for (int u = -radius; u <= radius; u++)
            {
                m_x += u * center[u];
            }
            int step = (int)im.step1();
            for (int v = 1; v <= radius; v++)
            {
                int d = xrange[v];
                int y_plus = 0;
                for (int u = -d; u <= d; u++)
                {
                    int pixel_b = center[u + step * v];
                    int pixel_u = center[u - step * v];
                    m_x += u * (pixel_u + pixel_b);
                    y_plus += (pixel_b - pixel_u);
                }
                m_y += v * y_plus;
            }
            kit->angle = fastAtan2((float)m_y, (float)m_x);
        }
    }

    void ORBFeature::compute_feature_num_in_pyramids(int feature_num_to_extract, double scale_factor, int nlevel)
    {
        // TODO
        float inv_factor = 1.0f / (scale_factor * scale_factor);
        // float inv_factor = 1.0f / scale_factor;
        float temp = feature_num_to_extract * (1 - inv_factor) / (1 - (float)pow((double)inv_factor, (double)nlevel));
        int sum_features = 0;
        for (int l = 0; l < nlevel - 1; l++)
        {
            mvi_feature_num_in_each_pyrimad[l] = cvRound(temp);
            sum_features += mvi_feature_num_in_each_pyrimad[l];
            temp *= inv_factor;
        }
        mvi_feature_num_in_each_pyrimad[nlevel - 1] = max(0, feature_num_to_extract - sum_features);
    }

    void ORBFeature::compute_pyramid(const Mat &im, int nlevel)
    {
        for (int l = 0; l < nlevel; l++)
        {
            float scale = mvd_inv_scale_factor[l];
            Size sz(cvRound((float)im.cols * scale), cvRound((float)im.rows * scale));
            Size wholeSize(sz.width + edge_threshold * 2, sz.height + edge_threshold * 2);
            Mat temp(wholeSize, im.type());
            mvm_image_pyramid[l] = temp(Rect(edge_threshold, edge_threshold, sz.width, sz.height));
            if (l != 0)
            {
                resize(mvm_image_pyramid[l - 1], mvm_image_pyramid[l], sz, 0, 0, INTER_LINEAR);
                copyMakeBorder(mvm_image_pyramid[l], temp, edge_threshold, edge_threshold, edge_threshold, edge_threshold, BORDER_REFLECT_101 + BORDER_ISOLATED);
            }
            else
            {
                copyMakeBorder(im, temp, edge_threshold, edge_threshold, edge_threshold, edge_threshold, BORDER_REFLECT101);
            }
        }
    }
    vector<Mat> ORBFeature::compute_pyramid_without_copyMakeBorder(const Mat &im, int nlevel)
    {
        vector<Mat> my_pyramid_image;
        my_pyramid_image.resize(8);
        for (int l = 0; l < nlevel; l++)
        {
            float scale = mvd_inv_scale_factor[l];
            Size sz(cvRound((float)im.cols * scale), cvRound((float)im.rows * scale));
            Mat temp(sz, im.type());
            my_pyramid_image[l] = temp;
            if (l != 0)
            {
                resize(my_pyramid_image[l - 1], my_pyramid_image[l], sz, 0, 0, INTER_LINEAR);
            }
            else
            {
                my_pyramid_image[l] = im.clone();
            }
        }
        return my_pyramid_image;
    }

    vector<KeyPoint> ORBFeature::DistribiteOctTree(const vector<KeyPoint> &vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY,
                                                   const int &N)
    {
        // img width>height,if width > height too much, split it.

        list<MyExtractorNode> lNodes;
        int nIni;
        if ((maxX - minX) >= (maxY - minY))
        {
            nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));
            const float hX = static_cast<float>(maxX - minX) / nIni;
            vector<MyExtractorNode *> vpIniNodes;
            vpIniNodes.reserve(nIni);
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(hX * static_cast<float>(i), 0);
                ni.UR = Point2i(hX * static_cast<float>(i + 1), 0);
                ni.BL = Point2i(ni.UL.x, maxY - minY);
                ni.BR = Point2i(ni.UR.x, maxY - minY);
                ni.vKeys.reserve(vToDistributeKeys.size());
                lNodes.push_back(ni);
                vpIniNodes[i] = &lNodes.back();
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.x / hX;
                vpIniNodes[index]->vKeys.push_back(kp);
            }
        }
        else
        {
            nIni = round(static_cast<float>(maxY - minY) / (maxX - minX));
            const float hY = static_cast<float>(maxY - minY) / nIni;
            vector<MyExtractorNode *> vpIniNodes;
            vpIniNodes.reserve(nIni);
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(0, hY * static_cast<float>(i));
                ni.BL = Point2i(0, hY * static_cast<float>(i + 1));
                ni.UR = Point2i(maxX - minX, ni.UL.y);
                ni.BR = Point2i(maxX - minX, ni.BL.y);
                ni.vKeys.reserve(vToDistributeKeys.size());
                lNodes.push_back(ni);
                vpIniNodes[i] = &lNodes.back();
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.y / hY;
                vpIniNodes[index]->vKeys.push_back(kp);
            }
        }
        int nNodesToExpand = 0;
        vector<pair<int, MyExtractorNode *>> v_size_and_node;
        v_size_and_node.reserve(4 * nNodesToExpand);
        list<MyExtractorNode>::iterator lit = lNodes.begin();
        while (lit != lNodes.end())
        {
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                lit++;
            }
            else if (lit->vKeys.empty())
            {
                lit = lNodes.erase(lit);
            }
            else
            {
                lit++;
                nNodesToExpand++;
                v_size_and_node.push_back(make_pair(lit->vKeys.size(), &(*lit)));
            }
        }
        int nNodes = nIni;

        bool bFinish = false;

        while (!bFinish)
        {
            if (nNodes + nNodesToExpand * 3 < N)
            {
                v_size_and_node.clear();
                nNodesToExpand = 0;
                lit = lNodes.begin();
                while (lit != lNodes.end())
                {
                    if (lit->bNoMore)
                    {
                        lit++;
                        continue;
                    }
                    else
                    {
                        MyExtractorNode n1, n2, n3, n4;
                        lit->DivideNode(n1, n2, n3, n4);
                        lit = lNodes.erase(lit);
                        nNodes--;
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            nNodes++;
                            if (n1.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            nNodes++;
                            if (n2.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            nNodes++;
                            if (n3.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            nNodes++;
                            if (n4.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                    }
                }
                if (nNodesToExpand == 0)
                {
                    bFinish = true;
                }
            }
            else
            {
                while (!bFinish)
                {
                    vector<pair<int, MyExtractorNode *>> tmp_v_size_and_node = v_size_and_node;
                    v_size_and_node.clear();
                    nNodesToExpand = 0;
                    sort(tmp_v_size_and_node.begin(), tmp_v_size_and_node.end());
                    for (int i = tmp_v_size_and_node.size() - 1; i >= 0; i--)
                    {
                        MyExtractorNode n1, n2, n3, n4;
                        tmp_v_size_and_node[i].second->DivideNode(n1, n2, n3, n4);

                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            nNodes++;
                            if (n1.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            nNodes++;
                            if (n2.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            nNodes++;
                            if (n3.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            nNodes++;
                            if (n4.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        lNodes.erase(tmp_v_size_and_node[i].second->lit);
                        nNodes--;
                        if (nNodes >= N)
                        {
                            bFinish = true;
                            break;
                        }
                    }
                    if (nNodesToExpand == 0)
                    {
                        bFinish = true;
                    }
                }
            }
        }

        vector<KeyPoint> vResultKeys;
        vResultKeys.reserve(N + 4);
        for (list<MyExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            vector<KeyPoint> &v_node_keys = lit->vKeys;
            KeyPoint *pKP = &v_node_keys[0];
            float max_respone = pKP->response;
            for (size_t k = 1; k < v_node_keys.size(); k++)
            {
                if (v_node_keys[k].response > max_respone)
                {
                    max_respone = v_node_keys[k].response;
                    pKP = &v_node_keys[k];
                }
            }
            vResultKeys.push_back(*pKP);
        }
        return vResultKeys;
    }
    void ORBFeature::Extract_FASTFeature_by_grid(const Mat &im, const float grid_size, vector<KeyPoint> &vToDistributeKeys, Rect &rect)
    {
        const int minBorderX = rect.x;
        const int minBorderY = rect.y;
        const int maxBorderX = rect.width + minBorderX;
        const int maxBorderY = rect.height + minBorderY;
        const float width = rect.width;
        const float height = rect.height;
        const int nCols = width / grid_size;
        const int nRows = height / grid_size;
        const int wCell = ceil(width / nCols);
        const int hCell = ceil(height / nRows);

        vToDistributeKeys.reserve(100000);
#ifdef _DEBUG
        const Scalar white = Scalar(255);
        const Scalar blue = Scalar(0, 255, 0);
        Mat img = im.clone();
        cvtColor(img, img, COLOR_GRAY2RGB);
#endif
        bool rowline_drawed = false;
        for (int i = 0; i < nRows; i++)
        {
            const float iniY = minBorderY + i * hCell;
            float maxY = iniY + hCell + 6;
            if (iniY >= maxBorderY - 6)
                continue;
            if (maxY > maxBorderY)
                maxY = maxBorderY;
#ifdef _DEBUG
            line(img, Point(minBorderX, minBorderY + i * hCell), Point(maxBorderX, minBorderY + i * hCell), blue, 1, 8, 0);
#endif
            for (int j = 0; j < nCols; j++)
            {

                const float iniX = minBorderX + j * wCell;
                float maxX = iniX + hCell + 6;
                if (iniX >= maxBorderX - 6)
                    continue;
                if (maxX > maxBorderX)
                    maxX = maxBorderX;
                vector<KeyPoint> vKeysCell;
                FAST(im.rowRange(iniY, maxY).colRange(iniX, maxX), vKeysCell, 20, true);
                if (vKeysCell.empty())
                {
                    FAST(im.rowRange(iniY, maxY).colRange(iniX, maxX), vKeysCell, 7, true);
                }
                if (!vKeysCell.empty())
                {
                    for (vector<KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                    {
                        (*vit).pt.x += j * wCell;
                        (*vit).pt.y += i * hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }
                if (!rowline_drawed)
                {
#ifdef _DEBUG
                    line(img, Point(minBorderX + j * wCell, minBorderY), Point(minBorderY + j * wCell, maxBorderY), blue, 1, 8, 0);
#endif
                }
            }
            rowline_drawed = true;
        }

#ifdef _DEBUG
        int n_kps = vToDistributeKeys.size();
        for (int i = 0; i < n_kps; i++)
        {
            circle(img, Point(vToDistributeKeys[i].pt.x + minBorderX, vToDistributeKeys[i].pt.y + minBorderY), 1, Scalar(0, 0, 250), 1);
        }
        line(img, Point(minBorderX, maxBorderY), Point(maxBorderX, maxBorderY), blue, 1, 8, 0);
        line(img, Point(maxBorderX, minBorderY), Point(maxBorderX, maxBorderY), blue, 1, 8, 0);
        Rect rect1(minBorderX + 3, minBorderY + 3, width - 6, height - 6);
        rectangle(img, rect1, blue, 1);

        imshow("image", img);
        waitKey();
        destroyWindow("image");
#endif

        // cout << "fast feature size after grid extract:" << vToDistributeKeys.size() << endl;
    }

    vector<KeyPoint> ORBFeature::Test_DistribiteOctTree(const Mat &im, const vector<KeyPoint> &vToDistributeKeys, Rect &rect,
                                                        const int &N)
    {
        // img width>height,if width > height too much, split it.
#ifdef _DEBUG
        Mat img = im.clone();
        cvtColor(img, img, COLOR_GRAY2RGB);
        const Scalar blue = Scalar(0, 255, 0);
        const Scalar red = Scalar(0, 0, 255);
        rectangle(img, rect, blue, 1);
#endif
        // clock_t start1, finish1, start2, finish2;
        // double duration;
        // start1 = clock();

        int minBorderX = rect.x;
        int minBorderY = rect.y;

        list<MyExtractorNode> lNodes;
        int nIni;
        if (rect.width >= rect.height)
        {
            nIni = round(static_cast<float>(rect.width) / rect.height);
            const float hX = static_cast<float>(rect.width) / nIni;
            vector<MyExtractorNode *> vpIniNodes;
            vpIniNodes.reserve(nIni);
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(hX * static_cast<float>(i), 0);
                ni.UR = Point2i(hX * static_cast<float>(i + 1), 0);
                ni.BL = Point2i(ni.UL.x, rect.height);
                ni.BR = Point2i(ni.UR.x, rect.height);
                ni.vKeys.reserve(vToDistributeKeys.size());
                lNodes.push_back(ni);
                vpIniNodes.push_back(&lNodes.back());
                if (i < nIni - 1)
                {
#ifdef _DEBUG
                    line(img, Point(ni.UR.x + minBorderX, ni.UR.y + minBorderY), Point(ni.BR.x + minBorderX, ni.BR.y + minBorderY), blue, 1, 8, 0);
#endif
                }
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.x / hX;
                vpIniNodes[index]->vKeys.push_back(kp);
            }
        }
        else
        {
            nIni = round(static_cast<float>(rect.height) / rect.width);
            const float hY = static_cast<float>(rect.height) / nIni;
            vector<MyExtractorNode *> vpIniNodes;
            vpIniNodes.reserve(nIni);
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(0, hY * static_cast<float>(i));
                ni.BL = Point2i(0, hY * static_cast<float>(i + 1));
                ni.UR = Point2i(rect.width, ni.UL.y);
                ni.BR = Point2i(rect.width, ni.BL.y);
                ni.vKeys.reserve(vToDistributeKeys.size());
                lNodes.push_back(ni);
                vpIniNodes.push_back(&lNodes.back());
                if (i < nIni - 1)
                {
#ifdef _DEBUG
                    line(img, Point(ni.BL.x + minBorderX, ni.BL.y + minBorderY), Point(ni.BR.x + minBorderX, ni.BR.y + minBorderY), blue, 1, 8, 0);
#endif
                }
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.y / hY;
                vpIniNodes[index]->vKeys.push_back(kp);
            }
        }
        int nNodesToExpand = 0;
        vector<pair<int, MyExtractorNode *>> v_size_and_node;
        v_size_and_node.reserve(4 * nNodesToExpand);
        list<MyExtractorNode>::iterator lit = lNodes.begin();
        while (lit != lNodes.end())
        {
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                lit++;
            }
            else if (lit->vKeys.empty())
            {
                lit = lNodes.erase(lit);
            }
            else
            {
                lit++;
                nNodesToExpand++;
                v_size_and_node.push_back(make_pair(lit->vKeys.size(), &(*lit)));
            }
        }
        int nNodes = nIni;

        bool bFinish = false;

        // start2 = clock();
        while (!bFinish)
        {
            if (nNodes + nNodesToExpand * 3 < N)
            {
                v_size_and_node.clear();
                nNodesToExpand = 0;
                lit = lNodes.begin();
                while (lit != lNodes.end())
                {
                    if (lit->bNoMore)
                    {
                        lit++;
                        continue;
                    }
                    else
                    {
                        MyExtractorNode n1, n2, n3, n4;
                        lit->DivideNode(n1, n2, n3, n4);
#ifdef _DEBUG
                        line(img, Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), blue, 1);
                        line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), blue, 1);
                        // imshow("image",img);
                        // waitKey();
                        // destroyWindow("image");
#endif

                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            nNodes++;
                            if (n1.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n1.UL.x + minBorderX, n1.UL.y + minBorderY), Point(n1.BR.x + minBorderX, n1.BR.y + minBorderY), red, 1);
                            line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            nNodes++;
                            if (n2.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n2.UL.x + minBorderX, n2.UL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), red, 1);
                            line(img, Point(n2.BL.x + minBorderX, n2.BL.y + minBorderY), Point(n2.UR.x + minBorderX, n2.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            nNodes++;
                            if (n3.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n3.UL.x + minBorderX, n3.UL.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), red, 1);
                            line(img, Point(n3.BL.x + minBorderX, n3.BL.y + minBorderY), Point(n3.UR.x + minBorderX, n3.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            nNodes++;
                            if (n4.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n4.UL.x + minBorderX, n4.UL.y + minBorderY), Point(n4.BR.x + minBorderX, n4.BR.y + minBorderY), red, 1);
                            line(img, Point(n4.BL.x + minBorderX, n4.BL.y + minBorderY), Point(n4.UR.x + minBorderX, n4.UR.y + minBorderY), red, 1);
#endif
                        }
                        lit = lNodes.erase(lit);
                        nNodes--;
                    }
                }
                if (nNodesToExpand == 0)
                {
                    bFinish = true;
                }
            }
            else
            {
                while (!bFinish)
                {
                    vector<pair<int, MyExtractorNode *>> tmp_v_size_and_node = v_size_and_node;
                    v_size_and_node.clear();
                    nNodesToExpand = 0;
                    sort(tmp_v_size_and_node.begin(), tmp_v_size_and_node.end());
                    for (int i = tmp_v_size_and_node.size() - 1; i >= 0; i--)
                    {
                        MyExtractorNode n1, n2, n3, n4;
                        tmp_v_size_and_node[i].second->DivideNode(n1, n2, n3, n4);
#ifdef _DEBUG
                        line(img, Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), blue, 1);
                        line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), blue, 1);
                        // imshow("image",img);
                        // waitKey();
                        // destroyWindow("image");
#endif
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            nNodes++;
                            if (n1.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n1.UL.x + minBorderX, n1.UL.y + minBorderY), Point(n1.BR.x + minBorderX, n1.BR.y + minBorderY), red, 1);
                            line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            nNodes++;
                            if (n2.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n2.UL.x + minBorderX, n2.UL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), red, 1);
                            line(img, Point(n2.BL.x + minBorderX, n2.BL.y + minBorderY), Point(n2.UR.x + minBorderX, n2.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            nNodes++;
                            if (n3.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n3.UL.x + minBorderX, n3.UL.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), red, 1);
                            line(img, Point(n3.BL.x + minBorderX, n3.BL.y + minBorderY), Point(n3.UR.x + minBorderX, n3.UR.y + minBorderY), red, 1);
#endif
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            nNodes++;
                            if (n4.vKeys.size() > 1)
                            {
                                nNodesToExpand++;
                                v_size_and_node.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        else
                        {
#ifdef _DEBUG
                            line(img, Point(n4.UL.x + minBorderX, n4.UL.y + minBorderY), Point(n4.BR.x + minBorderX, n4.BR.y + minBorderY), red, 1);
                            line(img, Point(n4.BL.x + minBorderX, n4.BL.y + minBorderY), Point(n4.UR.x + minBorderX, n4.UR.y + minBorderY), red, 1);
#endif
                        }
                        lNodes.erase(tmp_v_size_and_node[i].second->lit);
                        nNodes--;
                        if (nNodes >= N)
                        {
                            bFinish = true;
                            break;
                        }
                    }
                    if (nNodesToExpand == 0)
                    {
                        bFinish = true;
                    }
                }
            }
        }
        // finish2 = clock();
        // duration = (double)(finish2 - start2) / CLOCKS_PER_SEC;
        // cout << "octree by list, while loop time cost:" << duration << endl;

        vector<KeyPoint> vResultKeys;
        vResultKeys.reserve(N + 2);

        for (list<MyExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            vector<KeyPoint> &v_node_keys = lit->vKeys;
            KeyPoint *pKP = &v_node_keys[0];
            float max_respone = pKP->response;
            for (size_t k = 1; k < v_node_keys.size(); k++)
            {
                if (v_node_keys[k].response > max_respone)
                {
                    max_respone = v_node_keys[k].response;
                    pKP = &v_node_keys[k];
                }
            }
            pKP->pt.x = pKP->pt.x + minBorderX;
            pKP->pt.y = pKP->pt.y + minBorderY;
            vResultKeys.push_back(*pKP);
#ifdef _DEBUG
            circle(img, Point(pKP->pt.x, pKP->pt.y), 2, Scalar(0, 0, 250), 2);
#endif
        }
        // cout << "result feature num:" << vResultKeys.size() << endl;
#ifdef _DEBUG
        imshow("image", img);
        waitKey();
        destroyWindow("image");
#endif

        // finish1 = clock();
        // duration = (double)(finish1 - start1) / CLOCKS_PER_SEC;
        // cout << "octree by list, time cost:" << duration << endl;

        return vResultKeys;
    }
    bool operator<(MyExtractorNode n1, MyExtractorNode n2)
    {
        return n1.feature_size < n2.feature_size;
    }
    vector<KeyPoint> ORBFeature::Test_DistribiteOctTree_by_prior_queue(const Mat &im, const vector<KeyPoint> &vToDistributeKeys, Rect &rect,
                                                                       const int &N)
    {
        int minBorderX = rect.x;
        int minBorderY = rect.y;
#ifdef _DEBUG
        Mat img = im.clone();
        cvtColor(img, img, COLOR_GRAY2RGB);
        const Scalar green = Scalar(0, 255, 0);
        const Scalar red = Scalar(0, 0, 255);
        const Scalar blue = Scalar(255, 0, 0);
        rectangle(img, rect, blue, 1);
        int n_kps = vToDistributeKeys.size();

        for (int i = 0; i < n_kps; i++)
        {
            circle(img, Point(vToDistributeKeys[i].pt.x + minBorderX, vToDistributeKeys[i].pt.y + minBorderY), 1, Scalar(0, 0, 250), 1);
        }
#endif

        clock_t start1, finish1, start2, finish2;
        double duration;
        start1 = clock();

        priority_queue<MyExtractorNode> q1, q2;
        int nIni;
        if (rect.width >= rect.height)
        {
            nIni = round(static_cast<float>(rect.width) / rect.height);
            const float hX = static_cast<float>(rect.width) / nIni;
            MyExtractorNode nodes_array[nIni];
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(hX * static_cast<float>(i), 0);
                ni.UR = Point2i(hX * static_cast<float>(i + 1), 0);
                ni.BL = Point2i(ni.UL.x, rect.height);
                ni.BR = Point2i(ni.UR.x, rect.height);
                ni.vKeys.reserve(vToDistributeKeys.size());
                nodes_array[i] = ni;
                if (i < nIni - 1)
                {
#ifdef _DEBUG
                    line(img, Point(ni.UR.x + minBorderX, ni.UR.y + minBorderY), Point(ni.BR.x + minBorderX, ni.BR.y + minBorderY), blue, 1, 8, 0);
#endif
                }
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.x / hX;
                nodes_array[index].vKeys.push_back(kp);
                nodes_array[index].feature_size++;
            }
            for (int i = 0; i < nIni; i++)
            {
                q2.push(nodes_array[i]);
            }
        }
        else
        {
            nIni = round(static_cast<float>(rect.height) / rect.width);
            const float hY = static_cast<float>(rect.height) / nIni;
            MyExtractorNode nodes_array[nIni];
            for (int i = 0; i < nIni; i++)
            {
                MyExtractorNode ni;
                ni.UL = Point2i(0, hY * static_cast<float>(i));
                ni.BL = Point2i(0, hY * static_cast<float>(i + 1));
                ni.UR = Point2i(rect.width, ni.UL.y);
                ni.BR = Point2i(rect.width, ni.BL.y);
                ni.vKeys.reserve(vToDistributeKeys.size());
                nodes_array[i] = ni;
                if (i < nIni - 1)
                {
#ifdef _DEBUG
                    line(img, Point(ni.BL.x + minBorderX, ni.BL.y + minBorderY), Point(ni.BR.x + minBorderX, ni.BR.y + minBorderY), blue, 1, 8, 0);
#endif
                }
            }
            for (size_t i = 0; i < vToDistributeKeys.size(); i++)
            {
                const KeyPoint &kp = vToDistributeKeys[i];
                int index = kp.pt.y / hY;
                nodes_array[index].vKeys.push_back(kp);
                nodes_array[index].feature_size++;
            }
            for (int i = 0; i < nIni; i++)
            {
                q2.push(nodes_array[i]);
            }
        }
        int block_size = 0;
        int block_to_expand_size = 0;
        int feature_size;
        MyExtractorNode tmp_node;
        while (!q2.empty())
        {
            tmp_node = q2.top();
            feature_size = tmp_node.vKeys.size();
            if (feature_size > 0)
            {
                if (feature_size == 1)
                {
                    tmp_node.bNoMore = true;
                    q1.push(tmp_node);
                    block_size++;
                }
                else
                {
                    q1.push(tmp_node);
                    block_size++;
                    block_to_expand_size++;
                }
            }
            q2.pop();
        }
        priority_queue<MyExtractorNode> tmp_q;
        bool bFinish = false;
        bool b_near_stop = false;

        start2 = clock();
        while (!bFinish)
        {
            if (block_size + block_to_expand_size * 3 >= N)
            {
                b_near_stop = true;
            }
            block_to_expand_size = 0;
            while (!q1.empty())
            {
                tmp_node = q1.top();
                if (tmp_node.bNoMore)
                {
                    q2.push(tmp_node);
                    q1.pop();
                    while (!q1.empty())
                    {
                        tmp_node = q1.top();
                        q2.push(tmp_node);
                        q1.pop();
                    }
                    break;
                }
                MyExtractorNode n1, n2, n3, n4;
                tmp_node.DivideNode(n1, n2, n3, n4);
#ifdef _DEBUG
                line(img, Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), blue, 1);
                line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), blue, 1);
                // imshow("image", img);
                // waitKey();
                // destroyWindow("image");
#endif
                feature_size = n1.feature_size;
                if (feature_size > 0)
                {
                    q2.push(n1);
                    block_size++;
                    if (feature_size > 1)
                        block_to_expand_size++;
                }
                else
                {
#ifdef _DEBUG
                    line(img, Point(n1.UL.x + minBorderX, n1.UL.y + minBorderY), Point(n1.BR.x + minBorderX, n1.BR.y + minBorderY), red, 1);
                    line(img, Point(n1.BL.x + minBorderX, n1.BL.y + minBorderY), Point(n1.UR.x + minBorderX, n1.UR.y + minBorderY), red, 1);
#endif
                }
                feature_size = n2.feature_size;
                if (feature_size > 0)
                {
                    q2.push(n2);
                    block_size++;
                    if (feature_size > 1)
                        block_to_expand_size++;
                }
                else
                {
#ifdef _DEBUG
                    line(img, Point(n2.UL.x + minBorderX, n2.UL.y + minBorderY), Point(n2.BR.x + minBorderX, n2.BR.y + minBorderY), red, 1);
                    line(img, Point(n2.BL.x + minBorderX, n2.BL.y + minBorderY), Point(n2.UR.x + minBorderX, n2.UR.y + minBorderY), red, 1);
#endif
                }
                feature_size = n3.feature_size;
                if (feature_size > 0)
                {
                    q2.push(n3);
                    block_size++;
                    if (feature_size > 1)
                        block_to_expand_size++;
                }
                else
                {
#ifdef _DEBUG
                    line(img, Point(n3.UL.x + minBorderX, n3.UL.y + minBorderY), Point(n3.BR.x + minBorderX, n3.BR.y + minBorderY), red, 1);
                    line(img, Point(n3.BL.x + minBorderX, n3.BL.y + minBorderY), Point(n3.UR.x + minBorderX, n3.UR.y + minBorderY), red, 1);
#endif
                }
                feature_size = n4.feature_size;
                if (feature_size > 0)
                {
                    q2.push(n4);
                    block_size++;
                    if (feature_size > 1)
                        block_to_expand_size++;
                }
                else
                {
#ifdef _DEBUG
                    line(img, Point(n4.UL.x + minBorderX, n4.UL.y + minBorderY), Point(n4.BR.x + minBorderX, n4.BR.y + minBorderY), red, 1);
                    line(img, Point(n4.BL.x + minBorderX, n4.BL.y + minBorderY), Point(n4.UR.x + minBorderX, n4.UR.y + minBorderY), red, 1);
#endif
                }
                q1.pop();
                block_size--;
                if (b_near_stop)
                {
                    if (block_size >= N)
                    {
                        while (!q1.empty())
                        {
                            tmp_node = q1.top();
                            q2.push(tmp_node);
                            q1.pop();
                        }
                        bFinish = true;
                        break;
                    }
                }
            }
            // tmp_q = q2;
            // while (!tmp_q.empty())
            // {
            //     cout << tmp_q.top().feature_size << " ";
            //     tmp_q.pop();
            // }
            // cout << endl;
            if (block_to_expand_size == 0 || bFinish)
            {
                bFinish = true;
                continue;
            }
            tmp_q = q1;
            q1 = q2;
            q2 = tmp_q;
        }
        finish2 = clock();
        duration = (double)(finish2 - start2) / CLOCKS_PER_SEC;
        cout << "octree by priority queue, time cost:" << duration << endl;

        vector<KeyPoint> vResultKeys;
        vResultKeys.reserve(N + 4);
        while (!q2.empty())
        {
            tmp_node = q2.top();
            vector<KeyPoint> &v_node_keys = tmp_node.vKeys;
            KeyPoint *pKP = &v_node_keys[0];
            float max_respone = pKP->response;
            for (size_t k = 1; k < v_node_keys.size(); k++)
            {
                if (v_node_keys[k].response > max_respone)
                {
                    max_respone = v_node_keys[k].response;
                    pKP = &v_node_keys[k];
                }
            }
            pKP->pt.x += minBorderX;
            pKP->pt.y += minBorderY;
            vResultKeys.push_back(*pKP);
            q2.pop();
#ifdef _DEBUG
            circle(img, Point(pKP->pt.x, pKP->pt.y), 2, red, 2);
#endif
        }

#ifdef _DEBUG
        imshow("image", img);
        waitKey();
        destroyWindow("image");
#endif

        finish1 = clock();
        duration = (double)(finish1 - start1) / CLOCKS_PER_SEC;
        cout << "octree by priority queue, time cost:" << duration << endl;

        cout << "result_feature num:" << vResultKeys.size() << endl;
        return vResultKeys;
    }

    void ORBFeature::extract_orb_compute_descriptor(const Mat &im, int feature_num, vector<KeyPoint> &_orb_keypoints, Mat &_descriptors)
    {
        mvi_feature_num_in_each_pyrimad.resize(mi_nlevel);
        compute_feature_num_in_pyramids(feature_num, md_scale_factor, mi_nlevel);
        mvm_image_pyramid.resize(mi_nlevel);
        compute_pyramid(im, mi_nlevel);

        int sum_feature_size = 0;
        vector<Mat> all_level_descriptors(mi_nlevel);
        vector<vector<KeyPoint>> all_level_keypoints(mi_nlevel);

        for (int i = 0; i < mi_nlevel; i++)
        {
            vector<KeyPoint> keypoints_to_distribute;
            int width = mvm_image_pyramid[i].cols;
            int height = mvm_image_pyramid[i].rows;
            Rect rect(border, border, width - 2 * border, height - 2 * border);
            Extract_FASTFeature_by_grid(mvm_image_pyramid[i], grid_size, keypoints_to_distribute, rect);

            vector<KeyPoint> result_points_after_octree;

            result_points_after_octree = Test_DistribiteOctTree(mvm_image_pyramid[i], keypoints_to_distribute, rect, mvi_feature_num_in_each_pyrimad[i]);
            all_level_keypoints[i] = result_points_after_octree;
            int feature_size = result_points_after_octree.size();
            sum_feature_size += feature_size;
            if (feature_size != 0)
            {
                compute_centroid_orientation(mvm_image_pyramid[i], all_level_keypoints[i], mvi_xrange, half_patch_size);
                Mat descriptors = Mat::zeros(feature_size, 32, CV_8U);

                compute_descriptors(mvm_image_pyramid[i], all_level_keypoints[i], descriptors);
                all_level_descriptors[i] = descriptors;
            }
        }
        // TMP
        //  for (int i = 0; i < 8; i++)
        //  {
        //      Mat img = mvm_image_pyramid[i].clone();
        //      cvtColor(img, img, COLOR_GRAY2RGB);
        //      for (vector<KeyPoint>::iterator kit = all_level_keypoints[i].begin(); kit != all_level_keypoints[i].end(); kit++)
        //      {
        //          circle(img, Point(kit->pt.x, kit->pt.y), 2, Scalar(0, 0, 250), 2);
        //      }
        //      imshow("image", img);
        //      waitKey();
        //      destroyWindow("image");
        //  }

        _orb_keypoints = vector<KeyPoint>(sum_feature_size);
        _descriptors = Mat::zeros(sum_feature_size, 32, CV_8U);
        int mono_index = 0;
        int stero_index = sum_feature_size - 1;
        for (int i = 0; i < mi_nlevel; i++)
        {
            vector<KeyPoint> &temp_v_kepoints = all_level_keypoints[i];
            int temp_feature_num = (int)temp_v_kepoints.size();
            float scale = mvd_scale_factor[i];
            int row = 0;
            for (vector<KeyPoint>::iterator kit = temp_v_kepoints.begin(); kit != temp_v_kepoints.end(); kit++)
            {
                kit->octave = i;
                if (i != 0)
                {
                    kit->pt *= scale;
                }
                // TODO ?
                // if (kit->pt.x >= 0 && kit->pt.x <= 1000)
                // {
                //     _orb_keypoints.at(stero_index) = (*kit);
                //     all_level_descriptors[i].row(row).copyTo(_descriptors.row(stero_index));
                //     stero_index--;
                // }
                // else
                // {
                //     _orb_keypoints.at(mono_index) = (*kit);
                //     all_level_descriptors[i].row(row).copyTo(_descriptors.row(mono_index));
                //     mono_index++;
                // }
                _orb_keypoints.at(mono_index) = (*kit);
                all_level_descriptors[i].row(row).copyTo(_descriptors.row(mono_index));
                mono_index++;
                row++;
            }
        }

        // Mat img = mvm_image_pyramid[0].clone();
        // cvtColor(img, img, COLOR_GRAY2RGB);
        // for (vector<KeyPoint>::iterator kit = _orb_keypoints.begin(); kit != _orb_keypoints.end(); kit++)
        // {
        //     circle(img, Point(kit->pt.x, kit->pt.y), 2, Scalar(0, 0, 250), 2);
        // }
        // imshow("image", img);
        // waitKey();
        // destroyWindow("image");
    }

    void ORBFeature::undistort_keypoints(vector<KeyPoint> &keypoints, vector<KeyPoint> &undistoeted_keypoints, int feature_num)
    {
        if (mm_distcoef.at<float>(0) == 0.0)
        {
            undistoeted_keypoints = keypoints;
            return;
        }
        // TODO can be faster!
        Mat mat(feature_num, 2, CV_32F);
        for (int i = 0; i < feature_num; i++)
        {
            mat.at<float>(i, 0) = keypoints[i].pt.x;
            mat.at<float>(i, 1) = keypoints[i].pt.y;
        }
        mat = mat.reshape(2);
        undistortPoints(mat, mat, mm_camera_intrinsics, mm_distcoef, Mat(), mm_camera_intrinsics);
        mat = mat.reshape(1);
        undistoeted_keypoints.resize(feature_num);
        for (int i = 0; i < feature_num; i++)
        {
            KeyPoint kp = keypoints[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            undistoeted_keypoints[i] = kp;
        }
    }

    void ORBFeature::compute_image_bounds(const Mat &im, float &box_minx, float &box_miny, float &box_maxx, float &box_maxy)
    {
        if (mm_distcoef.at<float>(0) != 0.0)
        {
            // TODO
            Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = im.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = im.rows;
            mat.at<float>(3, 0) = im.cols;
            mat.at<float>(3, 1) = im.rows;
            mat = mat.reshape(2);
            undistortPoints(mat, mat, mm_camera_intrinsics, mm_distcoef, Mat(), mm_camera_intrinsics);
            mat = mat.reshape(1);

            box_minx = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            box_miny = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            box_maxx = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            box_maxy = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            box_minx = 0.0f;
            box_miny = 0.0f;
            box_maxx = im.cols;
            box_maxy = im.rows;
        }
    }


}