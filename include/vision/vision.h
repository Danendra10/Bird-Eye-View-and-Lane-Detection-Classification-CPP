#include "opencv2/opencv.hpp"

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
enum LaneClass
{
    LEFT_LANE,
    MIDDLE_LANE,
    RIGHT_LANE
};

void LaneDetect(Mat &frame_src, Mat &frame_dst, vector<vector<Point>> &lanes)
{
    static int lane_idx = 0;
    static int prev_lane_idx = 0;
    static int prev_lane_pos_x = 0;
    static int prev_lane_pos_y = 0;
    static int min_lane_points = 10; // Minimum number of points to consider a lane

    lanes.clear(); // Clear the lanes vector

    for (int x = 0; x < frame_src.cols; x++)
    {
        vector<Point> current_lane_points;                      // Temporary vector to store lane points for the current x
        int closest_point_index = -1;                           // Index of the closest lane point
        float min_distance = std::numeric_limits<float>::max(); // Minimum distance to the closest point
        LaneClass lane_class = MIDDLE_LANE;                     // Classification of the current lane

        for (int y = 0; y < frame_src.rows; y++)
        {
            if (prev_lane_pos_x == 0 && prev_lane_pos_y == 0)
            {
                prev_lane_pos_x = x;
                prev_lane_pos_y = y;
            }

            if (frame_src.at<uchar>(y, x) == 255)
            {
                Point lane_point(x, y);
                current_lane_points.push_back(lane_point);

                // Calculate the distance to the current lane point
                float distance = sqrt(pow(x - prev_lane_pos_x, 2) + pow(y - prev_lane_pos_y, 2));

                // Update the closest point index and minimum distance
                if (distance < min_distance)
                {
                    closest_point_index = current_lane_points.size() - 1;
                    min_distance = distance;
                }
            }
        }

        printf("x: %d, closest_point_index: %d, min_distance: %f size: %d\n", x, closest_point_index, min_distance, current_lane_points.size());

        if (current_lane_points.size() >= min_lane_points)
        {
            // Classify the lane based on the closest point index
            if (closest_point_index < current_lane_points.size() / 3)
                lane_class = LEFT_LANE;
            else if (closest_point_index > (current_lane_points.size() * 2) / 3)
                lane_class = RIGHT_LANE;

            lanes.push_back(current_lane_points);
        }
    }
}
