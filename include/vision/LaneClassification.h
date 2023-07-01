#ifndef LANE_CLASSIFICATION_H
#define LANE_CLASSIFICATION_H
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

enum LaneSides
{
    LEFT_LANE,
    MIDDLE_LANE,
    RIGHT_LANE
};

struct Lane
{
    LaneSides lane_side;
    Point lane_point;
    int lane_width;
    int lane_height;
    int lane_area;
    int lane_angle;
    int lane_slope;
    int lane_intercept;
    int lane_curvature;
    int lane_distance;
    int lane_confidence;
};

void ClassifyLanes(Mat &src, vector<Vec4i> &lines, vector<Lane> &Lanes)
{
    Lanes.clear();
    for (int x = 0; x < src.cols; x++)
    {
        for (int y = 0; y < src.rows; y++)
        {
        }
    }
}

void ClassifyLanes2(vector<Vec4i> &lines, vector<Lane> &Lanes)
{
    Lane prev_lane;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        Lane lane;
        if (prev_lane.lane_point.x == 0 && prev_lane.lane_point.y == 0)
        {
            prev_lane.lane_point.x = (int)((l[0] + l[2]) / 2);
            prev_lane.lane_point.y = (int)((l[1] + l[3]) / 2);
            prev_lane.lane_side = LEFT_LANE;
        }

        lane.lane_point = Point((int)((l[0] + l[2]) / 2), (int)((l[1] + l[3]) / 2));

        Point2f lane_diff = lane.lane_point - prev_lane.lane_point;
        float lane_angle = atan2(lane_diff.y, lane_diff.x) * 180 / CV_PI;
        float lane_slope = lane_diff.y / lane_diff.x;
        float lane_intercept = lane.lane_point.y - lane_slope * lane.lane_point.x;
        float lane_distance = sqrt(pow(lane.lane_point.x - prev_lane.lane_point.x, 2) + pow(lane.lane_point.y - prev_lane.lane_point.y, 2));

        printf("lane_angle: %f, lane_slope: %f, lane_intercept: %f, lane_distance: %f\n", lane_angle, lane_slope, lane_intercept, lane_distance);

        // conditions to decide the lane side
        if (lane_angle > 0 && lane_angle < 30 && lane_slope > 0)
        {
            lane.lane_side = LEFT_LANE;
        }
        else if (lane_angle > 30 && lane_angle < 60 && lane_slope < 0)
        {
            lane.lane_side = MIDDLE_LANE;
        }
        else if (lane_angle > 60 && lane_angle < 90)
        {
            lane.lane_side = RIGHT_LANE;
        }

        prev_lane = lane;
        Lanes.push_back(lane);
    }
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     Vec4i l = lines[i];
    //     Lane lane;
    //     lane.lane_point = Point(l[0], l[1]);
    //     lane.lane_width = l[2] - l[0];
    //     lane.lane_height = l[3] - l[1];
    //     lane.lane_area = lane.lane_width * lane.lane_height;
    //     lane.lane_angle = atan2(lane.lane_height, lane.lane_width) * 180 / CV_PI;
    //     lane.lane_slope = lane.lane_height / lane.lane_width;
    //     lane.lane_intercept = lane.lane_point.y - lane.lane_slope * lane.lane_point.x;
    //     lane.lane_curvature = 0;
    //     lane.lane_distance = 0;
    //     lane.lane_confidence = 0;

    //     if (lane.lane_angle > 0 && lane.lane_angle < 30)
    //     {
    //         lane.lane_side = LEFT_LANE;
    //     }
    //     else if (lane.lane_angle > 30 && lane.lane_angle < 60)
    //     {
    //         lane.lane_side = MIDDLE_LANE;
    //     }
    //     else if (lane.lane_angle > 60 && lane.lane_angle < 90)
    //     {
    //         lane.lane_side = RIGHT_LANE;
    //     }

    //     Lanes.push_back(lane);
    // }
}

#endif