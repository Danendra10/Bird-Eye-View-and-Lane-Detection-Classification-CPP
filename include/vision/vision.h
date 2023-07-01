#include "opencv2/opencv.hpp"
#include "fstream"

#include <Eigen/Dense>
#include <numeric>

using namespace Eigen;

using namespace std;
using namespace cv;

#define INPUT_WIDTH 800
#define INPUT_HEIGHT 800
#define OUTPUT_WIDTH 800
#define OUTPUT_HEIGHT 800

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)
#define DST_REMAPPED_WIDTH 800
#define DST_REMAPPED_HEIGHT 800
#define SRC_RESIZED_WIDTH 800
#define SRC_RESIZED_HEIGHT 800

int h_min = 0;
int h_max = 176;
int l_min = 64;
int l_max = 255;
int s_min = 0;
int s_max = 255;

string video_path = "../test.mp4";

int *maptable = new int[SRC_RESIZED_WIDTH * SRC_RESIZED_HEIGHT];

struct CameraParameters
{
    double horizontal_fov; // in radian
    double vertical_fov;
    int image_width;      // in pixels
    int image_height;     // in pixels
    double near_clip;     // in meters
    double far_clip;      // in meters
    double noise_mean;    // in meters
    double noise_std_dev; // in meters
    double hack_baseline; // in meters
    double distortion_k1; // radial distortion coefficient
    double distortion_k2; // radial distortion coefficient
    double distortion_k3; // radial distortion coefficient
    double distortion_t1; // tangential distortion coefficient
    double distortion_t2; // tangential distortion coefficient
    double camera_pos_x;  // in cm
    double camera_pos_y;  // in cm
    double camera_pos_z;  // in cm
    double principal_point_x;
    double principal_point_y;
    double translation_x;
    double translation_y;
    double vanishing_point_x;
    double vanishing_point_y;
};

CameraParameters cam_params;

void InitializeVariables();
void GenerateImpMapTable();
void InversePerspective1(int dst_w, int dst_h, int *dst, int src_w, int src_h, int *src);
void BuildIPMTable(const int src_w, const int src_h, const int dst_w, const int dst_h, const int vanishing_pt_x, const int vanishing_pt_y, int *maptable);
void InversePerspective(const int dst_w, const int dst_h, const unsigned char *src, const int *maptable, unsigned char *dst);
Mat ConvertToImage(int *src);
void DrawVanishingPoint(Mat *frame);
void ScanLines(vector<Point> &dst, Mat &frame_src);
// enum LaneClass
// {
//     LEFT_LANE,
//     MIDDLE_LANE,
//     RIGHT_LANE
// };

// struct LaneStruct
// {
//     Point lane_point;
//     uint8_t classified_as;
// };

// void LaneDetect1(Mat &frame_src, Mat &frame_dst, vector<LaneStruct> &lanes)
// {
//     static int lane_idx = 0;
//     static int prev_lane_idx = 0;
//     static int prev_lane_pos_x = 0;
//     static int prev_lane_pos_y = 0;
//     static int min_lane_points = 100; // Minimum number of points to consider a lane

//     lanes.clear(); // Clear the lanes vector

//     vector<Point> current_lane_points; // Temporary vector to store lane points for the current x
//     for (int x = 0; x < frame_src.rows; x++)
//     {
//         int closest_point_index = -1;                           // Index of the closest lane point
//         float min_distance = std::numeric_limits<float>::max(); // Minimum distance to the closest point
//         LaneStruct lane;                                        // Structure to store the lane information

//         for (int y = 0; y < frame_src.cols; y++)
//         {
//             if (prev_lane_pos_x == 0 && prev_lane_pos_y == 0)
//             {
//                 prev_lane_pos_x = x;
//                 prev_lane_pos_y = y;
//             }

//             if (frame_src.at<uchar>(y, x) == 255)
//             {
//                 Point lane_point(x, y);
//                 current_lane_points.push_back(lane_point);

//                 // Calculate the distance to the current lane point
//                 float distance = sqrt(pow(x - prev_lane_pos_x, 2) + pow(y - prev_lane_pos_y, 2));

//                 if (prev_lane_pos_x != x || prev_lane_pos_y != y)
//                 {
//                     prev_lane_pos_x = x;
//                     prev_lane_pos_y = y;
//                 }

//                 // Update the closest point index and minimum distance
//                 if (distance < min_distance)
//                 {
//                     closest_point_index = current_lane_points.size() - 1;
//                     min_distance = distance;
//                 }
//                 // printf("dist %f %f prev lane %d %d curr lane %d %d\n", distance, min_distance, prev_lane_pos_x, prev_lane_pos_y, x, y);

//                 // what is the closest point index? is it right? left? middle?
//             }
//         }

//         // printf("x: %d, closest_point_index: %d, min_distance: %f size: %d\n", x, closest_point_index, min_distance, current_lane_points.size());

//         if (current_lane_points.size() >= min_lane_points)
//         {
//             // Classify the lane based on the closest point index
//             if (closest_point_index < current_lane_points.size() / 3)
//                 lane.classified_as = LEFT_LANE;
//             else if (closest_point_index > (current_lane_points.size() * 2) / 3)
//                 lane.classified_as = RIGHT_LANE;
//             else
//                 lane.classified_as = MIDDLE_LANE;
//             printf("closest_point_index: %d, size: %d %f %f\n", closest_point_index, current_lane_points.size(), RAD2DEG(atan2(current_lane_points[closest_point_index].y - prev_lane_pos_y, current_lane_points[closest_point_index].x - prev_lane_pos_x)), RAD2DEG(atan2(current_lane_points[closest_point_index].y - prev_lane_pos_y, current_lane_points[closest_point_index].x - prev_lane_pos_x)));
//             lane.lane_point = current_lane_points[closest_point_index];

//             lanes.push_back(lane);
//         }
//     }
// }

// void LaneDetect(Mat &frame_src, Mat &frame_dst, vector<LaneStruct> &lanes)
// {
//     static int lane_idx = 0;
//     static int prev_lane_idx = 0;
//     static int prev_lane_pos_x = 0;
//     static int prev_lane_pos_y = 0;
//     static int min_lane_points = 100; // Minimum number of points to consider a lane
//     float lane_threshold = 10.0f;     // Threshold distance to classify the lane

//     lanes.clear(); // Clear the lanes vector

//     vector<Point> current_lane_points; // Temporary vector to store lane points for the current x
//     for (int x = 0; x < frame_src.rows; x++)
//     {
//         int closest_point_index = -1;                           // Index of the closest lane point
//         float min_distance = std::numeric_limits<float>::max(); // Minimum distance to the closest point
//         LaneStruct lane;                                        // Structure to store the lane information

//         for (int y = 0; y < frame_src.cols; y++)
//         {
//             if (prev_lane_pos_x == 0 && prev_lane_pos_y == 0)
//             {
//                 prev_lane_pos_x = x;
//                 prev_lane_pos_y = y;
//             }

//             if (frame_src.at<uchar>(y, x) == 255)
//             {
//                 Point lane_point(x, y);
//                 current_lane_points.push_back(lane_point);

//                 // Calculate the distance to the current lane point
//                 float distance = sqrt(pow(x - prev_lane_pos_x, 2) + pow(y - prev_lane_pos_y, 2));

//                 if (prev_lane_pos_x != x || prev_lane_pos_y != y)
//                 {
//                     prev_lane_pos_x = x;
//                     prev_lane_pos_y = y;
//                 }

//                 // Update the closest point index and minimum distance
//                 if (distance < min_distance)
//                 {
//                     closest_point_index = current_lane_points.size() - 1;
//                     min_distance = distance;
//                 }
//             }
//         }
//         printf("current_lane_points.size(): %d min_distance: %f\n", current_lane_points.size(), min_distance);
//         if (current_lane_points.size() >= min_lane_points)
//         {
//             lane.lane_point = current_lane_points[closest_point_index];

//             // Classify the lane based on the closest point distance
//             if (min_distance < lane_threshold)
//             {
//                 if (closest_point_index < current_lane_points.size() / 2)
//                 {
//                     printf("left lane\n");
//                     lane.classified_as = LEFT_LANE;
//                 }
//                 else
//                 {
//                     printf("right lane\n");
//                     lane.classified_as = RIGHT_LANE;
//                 }
//             }
//             else
//             {
//                 lane.classified_as = MIDDLE_LANE;
//             }

//             lanes.push_back(lane);
//         }
//     }
//     current_lane_points.clear();
// }

enum LaneClass
{
    LEFT_LANE,
    MIDDLE_LANE,
    RIGHT_LANE
};

struct LaneStruct
{
    Point lane_point;
    LaneClass classified_as;
    uint16_t closest_point_index;
};

vector<LaneStruct> Lanes;

void LaneDetect(Mat &frame_src, Mat &frame_dst, vector<LaneStruct> &lanes)
{
    static int prev_left_lane_pos_x = 0;
    static int prev_left_lane_pos_y = 0;
    static int prev_middle_lane_pos_x = 0;
    static int prev_middle_lane_pos_y = 0;
    static int prev_right_lane_pos_x = 0;
    static int prev_right_lane_pos_y = 0;

    lanes.clear();

    vector<Point> curr_lane;
    for (int x = 0; x < frame_src.cols; x++)
    {
        for (int y = 200; y < frame_src.rows - 50; y++)
        {
            LaneStruct lane;
            if (frame_src.at<uchar>(y, x) == 255)
            {
                lane.lane_point.x = x;
                lane.lane_point.y = y;
                lanes.push_back(lane);
            }
        }
    }
}

bool DetectPointIsCloseToAnotherPoint(Point &point_1, Point &point_2, float threshold)
{
    float distance = sqrt(pow(point_1.x - point_2.x, 2) + pow(point_1.y - point_2.y, 2));
    if (distance < threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void FillPolyFromPoints(Mat &frame, vector<LaneStruct> &points)
{
    vector<Point> poly_points;
    for (int i = 0; i < points.size(); i++)
    {
        poly_points.push_back(points[i].lane_point);
    }
    fillConvexPoly(frame, poly_points, Scalar(255, 255, 255));
}

void FillPolyFromMatrix(Mat &frame_in, Mat &frame_out)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(frame_in, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    drawContours(frame_out, contours, -1, Scalar(255, 255, 255), FILLED, 8, hierarchy);
}

void ClassifyLane(vector<LaneStruct> &lanes)
{
    static LaneClass lane_class = LEFT_LANE;
    static LaneClass prev_lane_class = MIDDLE_LANE;

    float dist_to_left_lane = FLT_MAX;
    float dist_to_middle_lane = FLT_MAX;
    float dist_to_right_lane = FLT_MAX;
    static uint8_t state_change_lane = 0;

    if (lanes.empty())
    {
        cout << "No lanes detected." << endl;
        return;
    }
    // ofstream myfile;
    // myfile.open("../lane.txt");
    lanes[0].classified_as = lane_class;
    for (int i = 1; i < lanes.size(); i++)
    {
        bool is_close_to_another_point = DetectPointIsCloseToAnotherPoint(lanes[i].lane_point, lanes[i - 1].lane_point, 10);
        printf("is_close_to_another_point: %d\n", is_close_to_another_point);
        if (is_close_to_another_point)
        {
            lanes[i].classified_as = lanes[i - 1].classified_as;
        }
        else
        {
            lanes[i].classified_as = MIDDLE_LANE;
        }
        // float slope = (lanes[i].lane_point.y - lanes[i - 1].lane_point.y) / (lanes[i].lane_point.x - lanes[i - 1].lane_point.x);
        // float distance = sqrt(pow(lanes[i].lane_point.x - lanes[i - 1].lane_point.x, 2) + pow(lanes[i].lane_point.y - lanes[i - 1].lane_point.y, 2));

        // // myfile << "slope: " << slope << " distance: " << distance << " X: " << lanes[i].lane_point.x << " Y: " << lanes[i].lane_point.y << " X prev: " << lanes[i - 1].lane_point.x << " Y prev: " << lanes[i - 1].lane_point.y << endl;

        // if (slope < -100 && distance > 120)
        // {
        //     state_change_lane = 1;
        // }

        // if (state_change_lane && lane_class == LEFT_LANE && prev_lane_class != LEFT_LANE)
        // {
        //     lane_class = MIDDLE_LANE;
        //     prev_lane_class = LEFT_LANE;
        //     state_change_lane = 0;
        // }

        // if (state_change_lane && lane_class == MIDDLE_LANE && prev_lane_class != MIDDLE_LANE)
        // {
        //     lane_class = LEFT_LANE;
        //     prev_lane_class = MIDDLE_LANE;
        //     state_change_lane = 0;
        // }
        // printf("State %d lane class %d\n", state_change_lane, lane_class);

        // lanes[i].classified_as = lane_class;

        // printf("slope: %f distance: %f X: %d Y: %d X prev: %d Y prev: %d\n", slope, distance, lanes[i].lane_point.x, lanes[i].lane_point.y, lanes[i - 1].lane_point.x, lanes[i - 1].lane_point.y);
    }
    // myfile.close();
}

void LaneDetect2(Mat &frame_src, Mat &frame_dst, vector<LaneStruct> &lanes)
{
    static int prev_lane_pos_x = 0;
    static int prev_lane_pos_y = 0;
    static int prev_left_lane_pos_x = 0;
    static int prev_left_lane_pos_y = 0;
    static int prev_middle_lane_pos_x = 0;
    static int prev_middle_lane_pos_y = 0;
    static int min_lane_points_thresh = 100;
    float lane_threshold_dist = 50.0f;
    float lane_threshold_dist_max = 100.0f;

    lanes.clear();

    vector<Point> current_lane_points;
    for (int x = 0; x < frame_src.cols; x++)
    {
        int closest_point_index = -1;
        float min_distance = FLT_MAX;
        LaneStruct lane;

        for (int y = 0; y < frame_src.rows; y++)
        {
            if (y >= 750)
                continue;
            if (prev_lane_pos_x == 0 && prev_lane_pos_y == 0)
            {
                prev_lane_pos_x = x;
                prev_lane_pos_y = y;
            }

            if (frame_src.at<uchar>(y, x) == 255)
            {
                if (prev_lane_pos_x == x && prev_lane_pos_y == y)
                    continue;
                current_lane_points.push_back(Point(x, y));

                float distance = sqrt(pow(x - prev_lane_pos_x, 2) + pow(y - prev_lane_pos_y, 2));

                if (prev_lane_pos_x != x || prev_lane_pos_y != y)
                {
                    prev_lane_pos_x = x;
                    prev_lane_pos_y = y;
                }
                if (prev_left_lane_pos_x == 0 && prev_left_lane_pos_y == 0)
                {
                    prev_left_lane_pos_x = x;
                    prev_left_lane_pos_y = y;
                }
                if (prev_middle_lane_pos_x == 0 && prev_middle_lane_pos_y == 0)
                {
                    prev_middle_lane_pos_x = x;
                    prev_middle_lane_pos_y = y;
                }

                float dist_to_left_lane = sqrt(pow(x - prev_left_lane_pos_x, 2) + pow(y - prev_left_lane_pos_y, 2));

                float dist_to_middle_lane = FLT_MAX;
                if (prev_middle_lane_pos_x != prev_left_lane_pos_x || prev_middle_lane_pos_y != prev_left_lane_pos_y)
                    dist_to_middle_lane = sqrt(pow(x - prev_middle_lane_pos_x, 2) + pow(y - prev_middle_lane_pos_y, 2));

                printf("dist_to_left_lane: %f dist to mid %f prev_left_lane_pos: %d %d prev right %d %d\n", dist_to_left_lane, dist_to_middle_lane, prev_left_lane_pos_x, prev_left_lane_pos_y, prev_middle_lane_pos_x, prev_middle_lane_pos_y);

                if (dist_to_left_lane < lane_threshold_dist && dist_to_middle_lane > lane_threshold_dist_max)
                {
                    printf("left lane\n");
                    lane.classified_as = LEFT_LANE;
                }
                else if (dist_to_middle_lane < lane_threshold_dist && dist_to_left_lane > lane_threshold_dist_max)
                {
                    printf("middle lane\n");
                    lane.classified_as = MIDDLE_LANE;
                }
                else
                {
                    printf("right lane\n");
                    lane.classified_as = RIGHT_LANE;
                }

                lane.closest_point_index = current_lane_points.size() - 1;
                lane.lane_point = current_lane_points[lane.closest_point_index];
                lanes.push_back(lane);

                if (lane.classified_as == LEFT_LANE && (prev_left_lane_pos_x != x || prev_left_lane_pos_y != y))
                {
                    printf("update left lane\n");
                    prev_left_lane_pos_x = x;
                    prev_left_lane_pos_y = y;
                }

                if (lane.classified_as == MIDDLE_LANE && (prev_middle_lane_pos_x != x || prev_middle_lane_pos_y != y))
                {
                    printf("update middle lane\n");
                    prev_middle_lane_pos_x = x;
                    prev_middle_lane_pos_y = y;
                }
            }
        }
    }
}

// void LaneDetect(Mat &frame_src, Mat &frame_dst, vector<LaneStruct> &lanes)
// {
//     static int prev_lane_pos_x = 0;
//     static int prev_lane_pos_y = 0;
//     static int min_lane_points = 100; // Minimum number of points to consider a lane
//     float lane_threshold = 10.0f;     // Threshold distance to classify the lane

//     lanes.clear(); // Clear the lanes vector

//     vector<Point> current_lane_points; // Temporary vector to store lane points for the current x
//     LaneClass lane_class = LEFT_LANE;  // Initialize the lane class as left

//     for (int x = 0; x < frame_src.cols; x++)
//     {
//         int closest_point_index = -1;                           // Index of the closest lane point
//         float min_distance = std::numeric_limits<float>::max(); // Minimum distance to the closest point
//         LaneStruct lane;                                        // Structure to store the lane information

//         for (int y = 0; y < frame_src.rows; y++)
//         {
//             if (prev_lane_pos_x == 0 && prev_lane_pos_y == 0)
//             {
//                 prev_lane_pos_x = x;
//                 prev_lane_pos_y = y;
//             }

//             if (frame_src.at<uchar>(y, x) == 255)
//             {
//                 Point lane_point(x, y);
//                 current_lane_points.push_back(lane_point);

//                 // Calculate the distance to the current lane point
//                 float distance = sqrt(pow(x - prev_lane_pos_x, 2) + pow(y - prev_lane_pos_y, 2));

//                 // Update the closest point index and minimum distance
//                 if (distance < min_distance)
//                 {
//                     closest_point_index = current_lane_points.size() - 1;
//                     min_distance = distance;
//                 }
//             }
//         }

//         if (current_lane_points.size() >= min_lane_points)
//         {
//             lane.lane_point = current_lane_points[closest_point_index];

//             // Classify the lane based on the difference between lane lines
//             if (x > 0)
//             {
//                 int diff_x = lane.lane_point.x - lanes.back().lane_point.x;
//                 if (diff_x > lane_threshold)
//                     lane_class = RIGHT_LANE;
//                 else if (diff_x < -lane_threshold)
//                     lane_class = LEFT_LANE;
//                 else
//                     lane_class = MIDDLE_LANE;
//             }

//             lane.classified_as = lane_class;
//             lanes.push_back(lane);
//         }

//         current_lane_points.clear();
//     }
// }

VectorXd polyfit(const VectorXd &x, const VectorXd &y, int degree)
{
    MatrixXd A(x.size(), degree + 1);
    VectorXd b = y;

    for (int i = 0; i < x.size(); i++)
    {
        for (int j = 0; j <= degree; j++)
        {
            A(i, j) = pow(x(i), degree - j);
        }
    }

    VectorXd coeffs = A.colPivHouseholderQr().solve(b);
    return coeffs;
}

std::vector<Vec4i> makePoints(Mat img, float slope, float intercept)
{
    int y1 = img.rows;
    int y2 = static_cast<int>(y1 * 3 / 5);
    int x1 = static_cast<int>((y1 - intercept) / slope);
    int x2 = static_cast<int>((y2 - intercept) / slope);
    return {Vec4i(x1, y1, x2, y2)};
}

// average slope intercept
vector<vector<Vec4i>> AverageSlopeIntercept(Mat img, std::vector<Vec4i> &lines)
{
    std::vector<Vec2f> left_fit, right_fit;

    // use polyfit to fit a line to the points
    VectorXd x(lines.size());
    VectorXd y(lines.size());

    for (int i = 0; i < lines.size(); i++)
    {
        x(i) = lines[i][0];
        y(i) = lines[i][1];
    }

    VectorXd coeffs = polyfit(x, y, 1);

    cout << "coeffs: " << coeffs << endl;
    float slope = coeffs[0];
    float intercept = coeffs[1];
    Vec2f left_fit_average = Vec2f(0, 0);
    Vec2f right_fit_average = Vec2f(0, 0);
    if (!left_fit.empty())
        left_fit_average = std::accumulate(left_fit.begin(), left_fit.end(), Vec2f(0, 0)) / static_cast<float>(left_fit.size());
    if (!right_fit.empty())
        right_fit_average = std::accumulate(right_fit.begin(), right_fit.end(), Vec2f(0, 0)) / static_cast<float>(right_fit.size());

    vector<Vec4i> left_line = makePoints(img, left_fit_average[0], left_fit_average[1]);
    vector<Vec4i> right_line = makePoints(img, right_fit_average[0], right_fit_average[1]);
    vector<vector<Vec4i>> lines_to_draw = {left_line, right_line};
    return lines_to_draw;
}

void DisplayLine(Mat *img_dst, vector<vector<Vec4i>> lines)
{
    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = 0; j < lines[i].size(); j++)
        {
            Vec4i l = lines[i][j];
            line(*img_dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
        }
    }
}

void LaneDetectHough(Mat &frame_src, Mat &frame_dst, vector<Vec4i> &lines)
{
    // Convert the image to grayscale
    Mat gray;
    cvtColor(frame_src, gray, COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    Mat blurred;
    GaussianBlur(gray, blurred, Size(5, 5), 0);

    imshow("blurred", blurred);

    Mat edges;
    Canny(blurred, edges, 10, 100);

    // Create a mask to exclude specific regions
    Mat mask = Mat::zeros(edges.size(), CV_8UC1);

    // Define the regions to ignore
    Point region1[] = {Point(4, 440), Point(296, 795), Point(0, 800)};
    Point region2[] = {Point(795, 442), Point(506, 795), Point(800, 800)};

    // Draw the ignored regions as white on the mask
    fillConvexPoly(mask, region1, 3, Scalar(255));
    fillConvexPoly(mask, region2, 3, Scalar(255));

    // Ignore rows 750-800
    Rect ignoreRegion(0, 750, mask.cols, 50);
    mask(ignoreRegion) = 255;
    // Apply the mask to the edges image
    edges.setTo(0, mask);

    imshow("edges", edges);

    HoughLinesP(edges, lines, 1, CV_PI / 180, 5, 5, 5);

    vector<vector<Vec4i>> avg_lines = AverageSlopeIntercept(frame_src, lines);

    DisplayLine(&frame_src, avg_lines);

    // Create a copy of the source frame for drawing the detected lines
    frame_dst = frame_src.clone();
}

void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        printf("Left button of the mouse is clicked - position (%d, %d)\n", x, y);
        Point *p = (Point *)userdata;
        p->x = x;
        p->y = y;
    }
}