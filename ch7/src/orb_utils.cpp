#include "orb_utils.hpp"
 
// FAST
const int cycle3_num = 16;
int dx[] = {-3, -3, -2, -1, 0, 1, 2, 3, 3,  3,  2,  1,  0, -1, -2, -3};
int dy[] = { 0,  1,  2,  3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};

// -------------------------------------------------------------------------------------------------- //
// ORB pattern
int ORB_pattern[256 * 4] = {
  8, -3, 9, 5/*mean (0), correlation (0)*/,
  4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
  -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
  7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
  2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
  1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
  -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
  -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
  -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
  10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
  -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
  -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
  7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
  -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
  -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
  -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
  12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
  -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
  -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
  11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
  4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
  5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
  3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
  -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
  -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
  -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
  -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
  -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
  -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
  5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
  5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
  1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
  9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
  4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
  2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
  -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
  -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
  4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
  0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
  -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
  -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
  -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
  8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
  0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
  7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
  -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
  10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
  -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
  10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
  -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
  -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
  3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
  5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
  -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
  3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
  2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
  -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
  -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
  -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
  -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
  6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
  -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
  -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
  -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
  3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
  -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
  -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
  2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
  -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
  -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
  5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
  -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
  -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
  -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
  10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
  7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
  -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
  -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
  7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
  -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
  -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
  -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
  7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
  -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
  1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
  2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
  -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
  -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
  7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
  1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
  9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
  -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
  -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
  7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
  12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
  6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
  5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
  2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
  3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
  2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
  9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
  -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
  -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
  1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
  6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
  2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
  6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
  3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
  7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
  -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
  -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
  -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
  -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
  8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
  4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
  -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
  4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
  -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
  -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
  7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
  -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
  -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
  8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
  -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
  1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
  7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
  -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
  11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
  -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
  3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
  5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
  0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
  -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
  0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
  -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
  5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
  3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
  -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
  -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
  -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
  6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
  -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
  -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
  1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
  4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
  -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
  2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
  -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
  4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
  -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
  -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
  7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
  4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
  -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
  7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
  7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
  -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
  -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
  -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
  2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
  10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
  -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
  8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
  2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
  -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
  -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
  -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
  5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
  -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
  -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
  -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
  -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
  -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
  2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
  -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
  -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
  -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
  -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
  6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
  -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
  11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
  7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
  -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
  -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
  -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
  -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
  -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
  -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
  -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
  -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
  1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
  1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
  9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
  5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
  -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
  -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
  -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
  -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
  8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
  2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
  7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
  -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
  -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
  4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
  3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
  -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
  5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
  4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
  -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
  0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
  -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
  3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
  -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
  8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
  -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
  2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
  10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
  6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
  -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
  -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
  -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
  -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
  -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
  4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
  2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
  6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
  3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
  11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
  -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
  4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
  2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
  -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
  -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
  -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
  6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
  0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
  -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
  -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
  -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
  5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
  2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
  -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
  9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
  11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
  3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
  -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
  3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
  -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
  5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
  8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
  7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
  -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
  7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
  9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
  7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
  -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
};

// pixel2camera
cv::Point2f pixel2camera(Point p, cv::Mat K) {
    return cv::Point2f(
        (p.first - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.second - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

// contraint point x~[0, rows-1] y~[0, cols-1]
void Constraint(Point& p, int rows, int cols) {
    if (p.first < 0) p.first = 0;
    if (p.first >= cols) p.first = cols - 1;
    if (p.second < 0) p.second = 0;
    if (p.second >= rows) p.second = rows - 1;
}

// Compute Hamming Distance for Descriptor
int HammingDistance(Descriptor desc_1, Descriptor desc_2) {
    int count = 0;
    for (int i=0;i<8;i++) {
        // uint32_t d_xor = desc_1.desc[i] ^ desc_2.desc[i];
        count += _mm_popcnt_u32(desc_1.desc[i] ^ desc_2.desc[i]);

        /*
        for (int j=0;j<32;j++) {
            if (d_xor & (1<<j)) count++;
        }
        */
    }

    return count;
}

/*
int MaxCountOne(bool* a, int len) {
    if (a == nullptr) {
        printf("[ERROR] Pointer is nullptr in MaxCountOne. \n");
        return -1;
    }

    int count = 0;
    int max_count = -1;
    bool flag = true;
    for (int i=0;i<len;i++) {
        if (a[i]) count++;
        else {
            flag = false;
            max_count = max(max_count, count);
            count = 0;
        }
    }

    if (flag) return len;

    // the last one is not equal to zero
    if (count) {
        int idx = 0;
        while (idx < cycle3_num && a[idx]) {
            count++;
            idx++;
        }
    }
    max_count = max(max_count, count);
    return max_count;
}
*/

// FAST algorithm
bool FeatureExtract(cv::Mat image, vector<mKeyPoint>& keypoints, int radius, double threshold, int patch_size) {
    cv::Mat image_gray = image.clone();
    if (image.channels() > 1) {
        /*
        printf("[ERROR] Feature Extract just process image with 1 channel. (GRAY_IMAGE)\n");
        printf("        Image channels: %d \n", image.channels());
        return false;
        */
        cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    }
    
    int rows = image_gray.rows;
    int cols = image_gray.cols;

    // Extract conner for each pixel
    for (int x=patch_size/2;x<cols-(patch_size/2);x++) {
        for (int y=patch_size/2;y<rows-(patch_size/2);y++) {
            mKeyPoint keypoint;

            // transverse all points on the cycle
            unsigned char center_pixel = image_gray.at<uchar>(y, x);

            // cnt[i]: from ith to left, the sum of points which satisfied
            int cnt[cycle3_num] = {0};
            int idxZero = -1;
            for (int i=0;i<cycle3_num;i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                unsigned char pixel = image_gray.at<uchar>(ny, nx);
                if (abs(center_pixel - pixel) > threshold) {
                    if (!i) cnt[i] = 1;
                    else cnt[i] = cnt[i-1] + 1;
                }
                else if (idxZero == -1) idxZero = i;
            }
            
            cnt[idxZero - 1] += cnt[cycle3_num - 1];
            if (*(max_element(cnt, cnt + cycle3_num)) < radius) continue;
            
            keypoint.SetPt(Point(x, y));

            // Compute orientation
            double m10 = 0.0;
            double m01 = 0.0;
            // double m00 = 0.0;

            for (int i=-patch_size/2;i<patch_size/2;i++) {
                for (int j=-patch_size/2;j<patch_size/2;j++) {
                    int nx = x + i;
                    int ny = y + i;

                    unsigned char pixel = image_gray.at<uchar>(ny, nx);
                    m10 += i * pixel;
                    m01 += j * pixel;
                }
            }

            double m_sqrt = sqrt(m01 * m01 + m10 * m10);
            keypoint.SetCos(m10/m_sqrt);
            keypoint.SetSin(m01/m_sqrt);

            keypoints.push_back(keypoint);
        }
    }

    return true;
}

// Show KeyPoints
void ShowKeyPoints(string gragh_name, cv::Mat image, vector<mKeyPoint>& keypoints, cv::Scalar color) {

    int k_size = keypoints.size();
    for (int i=0;i<k_size;i++) {
        int x = keypoints[i].GetPt().first;
        int y = keypoints[i].GetPt().second;

        cv::circle(image, cv::Point(x, y), 3, color);
    }

    cv::imshow(gragh_name, image);
    return ;
}


void ComputeORB(cv::Mat image, vector<mKeyPoint> keypoints, vector<Descriptor>& descriptor) {
    cv::Mat image_gray = image.clone();
    if (image.channels() > 1) {
        /*
        printf("[ERROR] Feature Extract just process image with 1 channel. (GRAY_IMAGE)\n");
        printf("        Image channels: %d \n", image.channels());
        return false;
        */
        cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    }

    int k_size = keypoints.size();
    for (int j=0;j<k_size;j++) {
        mKeyPoint keypoint = keypoints[j];
        Descriptor desc;
        desc.Zero();
        for (int i=0;i<8;i++) {
            uint32_t d = 0;
            for (int k=0;k<32;k++) {
                int bit = i * 32 + k;

                Point p(ORB_pattern[bit * 4], ORB_pattern[bit * 4 + 1]);
                Point q(ORB_pattern[bit * 4 + 2], ORB_pattern[bit * 4 + 3]);

                // rotate with orientation
                Point pp((int)(keypoint.GetCos() * p.first - keypoint.GetSin() * p.second + keypoint.GetPt().first),
                        (int)(keypoint.GetSin() * p.first + keypoint.GetCos() * p.second + keypoint.GetPt().second));
                Point qq((int)(keypoint.GetCos() * q.first - keypoint.GetSin() * q.second + keypoint.GetPt().first), 
                        (int)(keypoint.GetSin() * q.first + keypoint.GetCos() * q.second + keypoint.GetPt().second));

                // Constraint(pp, image_gray.rows, image_gray.cols);
                // Constraint(qq, image_gray.rows, image_gray.cols);

                if (image_gray.at<uchar>(pp.second, pp.first) < image_gray.at<uchar>(qq.second, qq.first)) {
                    d |= 1 << k;
                }
            }
            desc.desc[i] = d;
        }
        descriptor.push_back(desc);
    }

    return ;
}

// Brute Match
void Match(vector<Descriptor> descriptors_1, vector<Descriptor> descriptors_2, vector<cv::DMatch>& matches, int d_max) {
    int size_1 = descriptors_1.size();
    int size_2 = descriptors_2.size();

    printf("[INFO] Descriptor_1 Size: %d\n", size_1);
    printf("[INFO] Descriptor_2 Size: %d\n", size_2);

    for (int i=0;i<size_1;i++) {
        cv::DMatch match{i, 0, 256};
        Descriptor desc_1 = descriptors_1[i];
        for (int j=0;j<size_2;j++) {
            Descriptor desc_2 = descriptors_2[j];

            int dist = HammingDistance(desc_1, desc_2);
            if (dist <= d_max && dist <= match.distance) {
                match.trainIdx = j;
                match.distance = dist;
            }
        }

        // printf("%d - %d: %f\n", match.queryIdx, match.trainIdx, match.distance);
        if (match.distance < d_max) 
            matches.push_back(match);
    }

    printf("[INFO] Match Points: %d\n", (int)matches.size());
}

// Show Match
void ShowMatch(string gragh_name, cv::Mat image_1, vector<mKeyPoint> keypoints_1, cv::Mat image_2, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches) {
    cv::Mat img_merge;
    cv::hconcat(image_1, image_2, img_merge);

    printf("[INFO] Matches Size: %d\n", (int)matches.size());

    for (int i=0;i<matches.size();i++) {
        printf("[INFO] (%d, %d) - (%d, %d): %f\n", keypoints_1[matches[i].queryIdx].GetPt().first, keypoints_1[matches[i].queryIdx].GetPt().second,
                                                keypoints_2[matches[i].trainIdx].GetPt().first, keypoints_2[matches[i].trainIdx].GetPt().second, matches[i].distance);
    }

    // 默认图像1和图像2，同大小
    int rows = image_1.rows;
    int cols = image_1.cols;

    // Draw KeyPoints in image_1
    int k_size = keypoints_1.size();
    for (int i=0;i<k_size;i++) {
        cv::Point p(keypoints_1[i].GetPt().first, keypoints_1[i].GetPt().second);
        cv::circle(img_merge, p, 3, cv::Scalar(0, 0, 0));
    }

    // Draw KeyPoints in image_2
    k_size = keypoints_2.size();
    for (int i=0;i<k_size;i++) {
        cv::Point p(keypoints_2[i].GetPt().first + cols, keypoints_2[i].GetPt().second);
        cv::circle(img_merge, p, 3, cv::Scalar(0, 0, 0));
    }

    // Draw lines between keypoints
    int m_size = matches.size();
    for (int i=0;i<m_size;i++) {
        cv::Point p(keypoints_1[matches[i].queryIdx].GetPt().first, keypoints_1[matches[i].queryIdx].GetPt().second);
        // cv::Point q(keypoints_1[matches[i].queryIdx].GetPt().first, keypoints_1[matches[i].queryIdx].GetPt().second);
        cv::Point q(keypoints_2[matches[i].trainIdx].GetPt().first + cols, keypoints_2[matches[i].trainIdx].GetPt().second);
        cv::line(img_merge, p, q, cv::Scalar(0, 0, 0));
    }

    cv::imshow(gragh_name, img_merge);
}

// pose estimate
void PoseEstimate2d2d(vector<mKeyPoint> keypoints_1, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches, cv::Mat& R, cv::Mat& t) {
    // camera calib
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // Convert mKeyPoint into cv::Point2f
    vector<cv::Point2f> points_1;
    vector<cv::Point2f> points_2;

    for (cv::DMatch m: matches) {
        points_1.push_back(cv::Point2f(keypoints_1[m.queryIdx].GetPt().first, keypoints_1[m.queryIdx].GetPt().second));
        points_2.push_back(cv::Point2f(keypoints_2[m.trainIdx].GetPt().first, keypoints_2[m.trainIdx].GetPt().second));
    }

    // Compute Fundamental Matrix
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points_1, points_2, CV_FM_8POINT);
    cout << "\n[INFO] fundamental matrix: \n" << fundamental_matrix << endl;

    // Compute Essential Matrix
    cv::Mat essential_matrix;
    cv::Point2d principal_point(325.1, 249.7);
    double focal_length = 521;
    essential_matrix = cv::findEssentialMat(points_1, points_2, focal_length, principal_point);
    cout << "\n[INFO] essential matrix: \n" << essential_matrix << endl;

    // Compute Homography
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points_1, points_2, cv::RANSAC, 3);
    cout << "\n[INFO] homography_matrix: \n" << homography_matrix << endl;

    // Get R and t
    cv::recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);
    cout << "\n[INFO] R: \n" << R << endl;
    cout << "[INFO] t: \n" << t << endl;

}

// triangular measurement
void Triangulation(vector<mKeyPoint> keypoints_1, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches, cv::Mat R, cv::Mat t, vector<cv::Point3d>& points) {
    // Translation Matrix of camera 1
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0);
    // Translation Matrix of camera 2
    cv::Mat T2 = (cv::Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    // camera calib
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<cv::Point2f> points_1, points_2;
    for (cv::DMatch m: matches) {
        points_1.push_back(pixel2camera(keypoints_1[m.queryIdx].GetPt(), K));
        points_2.push_back(pixel2camera(keypoints_2[m.trainIdx].GetPt(), K));
    }

    cv::Mat point4d;
    cv::triangulatePoints(T1, T2, points_1, points_2, point4d);

    cout << "\n[INFO] point4d: \n" << point4d << endl;

    // Convert to vector<Point3d>
    for (int i=0;i<point4d.cols;i++) {
        cv::Mat x = point4d.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d point(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(point);
    }
}

// Draw Cloud
void DrawCloud(vector<cv::Point3d> points) {
    if (points.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    // Create pangolin window and plot
    pangolin::CreateWindowAndBind("Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/ 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        

        glPointSize(2);
        glBegin(GL_POINTS);
        for (cv::Point3d p: points) {
            glColor3f(0.0f, 0.0f, 0.0f);
            glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }
}

// Draw KeyPoints
void DrawKeypoints(string gname, cv::Mat image, vector<mKeyPoint> keypoints, int radius, cv::Scalar color) {
    for (mKeyPoint p: keypoints) {
        cv::circle(image, cv::Point(p.GetPt().first, p.GetPt().second), radius, color);
    }

    cv::imshow(gname, image);
}