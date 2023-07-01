#include "opencv2/opencv.hpp"
#include "fstream"

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

void ClassifyLane(vector<LaneStruct> &lanes)
{
    LaneClass lane_class = LEFT_LANE;

    float dist_to_left_lane = FLT_MAX;
    float dist_to_middle_lane = FLT_MAX;
    float dist_to_right_lane = FLT_MAX;

    if (lanes.empty())
    {
        cout << "No lanes detected." << endl;
        return;
    }
    ofstream myfile;
    myfile.open("../lane.txt");
    for (int i = 0; i < lanes.size(); i++)
    {
        float slope = (lanes[i].lane_point.y - lanes[i - 1].lane_point.y) / (lanes[i].lane_point.x - lanes[i - 1].lane_point.x);
        float distance = sqrt(pow(lanes[i].lane_point.x - lanes[i - 1].lane_point.x, 2) + pow(lanes[i].lane_point.y - lanes[i - 1].lane_point.y, 2));

        myfile << "slope: " << slope << " distance: " << distance << " X: " << lanes[i].lane_point.x << " Y: " << lanes[i].lane_point.y << " X prev: " << lanes[i - 1].lane_point.x << " Y prev: " << lanes[i - 1].lane_point.y << endl;

        printf("slope: %f distance: %f X: %d Y: %d X prev: %d Y prev: %d\n", slope, distance, lanes[i].lane_point.x, lanes[i].lane_point.y, lanes[i - 1].lane_point.x, lanes[i - 1].lane_point.y);
    }
    myfile.close();
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