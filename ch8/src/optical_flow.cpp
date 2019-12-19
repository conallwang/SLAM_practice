#include <orb_utils.hpp>

string img_1 = "/home/johnson/SLAM/ch8/pics/LK1.png";
string img_2 = "/home/johnson/SLAM/ch8/pics/LK2.png";

void ShowKeypoints(string gname, cv::Mat image, vector<cv::Point2f> points, int radius = 3, cv::Scalar color = cv::Scalar(0, 0, 0)) {
    for (cv::Point2f p: points) {
        cv::circle(image, cv::Point(p.x, p.y), radius, color);
    }
    cv::imshow(gname, image);
}

void ShowFlow(string gname, cv::Mat image, vector<cv::Point2f> points_1, vector<cv::Point2f> points_2, int radius = 2,
    cv::Scalar color_1 = cv::Scalar(0, 0, 0), cv::Scalar color_2 = cv::Scalar(255, 255, 255)) {
    if (points_1.size() != points_2.size()) {
        cout << "[ERROR] points num is diff!" << endl;
        return ;
    }

    int p_size = points_1.size();
    for (int i=0;i<p_size;i++) {
        auto p_1 = points_1[i];
        auto p_2 = points_2[i];

        cv::circle(image, cv::Point(p_1.x, p_1.y), radius, color_1);
        cv::circle(image, cv::Point(p_2.x, p_2.y), radius, color_2);
        cv::line(image, cv::Point(p_1.x, p_1.y), cv::Point(p_2.x, p_2.y), color_1);
    }

    cv::imshow(gname, image);
}

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    cout << "[INFO] IMAGE_PATH_1: " << img_1 << endl;
    cout << "[INFO] IMAGE_PATH_2: " << img_2 << endl;

    // read images
    cout << "\n[STATUS] Starting read images ..." << endl;
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);

    // extract points
    cout << "\n[STATUS] Starting Feature Extracting ..." << endl;
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;

    // use opencv for validation
    vector<cv::Point2f> points_1, points_2;
    for (mKeyPoint p: keypoints_1) {
        points_1.push_back(cv::Point2f(p.GetPt().first, p.GetPt().second));
    }
    vector<uchar> status;
    vector<float> error;

    cout << "\n[STATUS] Starting LK optical flow ..." << endl;
    cv::calcOpticalFlowPyrLK(image_1, image_2, points_1, points_2, status, error);

    ShowKeypoints("result", image_2, points_2);
    ShowFlow("flow", image_2, points_1, points_2, 1);
    cv::waitKey(0);

    return 0;
}