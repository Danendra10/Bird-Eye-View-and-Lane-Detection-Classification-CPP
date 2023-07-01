#include "vision/vision.h"

int main()
{
    // slider hls
    namedWindow("thresholding", WINDOW_AUTOSIZE);
    createTrackbar("H min", "thresholding", &h_min, 255);
    createTrackbar("H max", "thresholding", &h_max, 255);
    createTrackbar("L min", "thresholding", &l_min, 255);
    createTrackbar("L max", "thresholding", &l_max, 255);
    createTrackbar("S min", "thresholding", &s_min, 255);
    createTrackbar("S max", "thresholding", &s_max, 255);

    InitializeVariables();
    VideoCapture cap(video_path);

    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    BuildIPMTable(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH >> 1, DST_REMAPPED_HEIGHT >> 1, maptable);

    while (1)
    {
        Mat raw_frame;
        cap >> raw_frame;

        if (raw_frame.empty())
            break;

        Mat frame_resize;
        Mat frame_gray_resize;
        Mat frame_remapped = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);

        resize(raw_frame, frame_resize, Size(cam_params.image_width, cam_params.image_height));
        cvtColor(frame_resize, frame_gray_resize, COLOR_BGR2GRAY);

        InversePerspective(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, frame_gray_resize.data, maptable, frame_remapped.data);
        line(raw_frame, Point(cam_params.image_width >> 1, 0), Point(cam_params.image_width >> 1, cam_params.image_height), Scalar(0, 0, 255), 1);
        line(raw_frame, Point(0, cam_params.image_height >> 1), Point(cam_params.image_width, cam_params.image_height >> 1), Scalar(0, 0, 255), 1);
        vector<vector<Point>> lines;

        Mat line_bgr;
        cvtColor(frame_remapped, line_bgr, COLOR_GRAY2BGR);
        Mat line_hls;
        cvtColor(line_bgr, line_hls, COLOR_BGR2HLS);

        inRange(line_hls, Scalar(h_min, l_min, s_min), Scalar(h_max, l_max, s_max), line_hls);
        dilate(line_hls, line_hls, Mat(), Point(-1, -1), 2);
        erode(line_hls, line_hls, Mat(), Point(-1, -1), 2);
        // ScanLines(lines, line_hls);

        LaneDetect(line_hls, frame_remapped, lines);

        for (int i = 0; i < lines.size(); i++)
        {
            circle(line_bgr, lines[i][LEFT_LANE], 1, Scalar(0, 0, 255), 1);
            circle(line_bgr, lines[i][MIDDLE_LANE], 1, Scalar(0, 255, 0), -1);
            circle(line_bgr, lines[i][RIGHT_LANE], 1, Scalar(0, 255, 0), -1);
        }
        imshow("Remapped", line_bgr);
        imshow("HLS", line_hls);

        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }
    return 0;
}

void InitializeVariables()
{
    cam_params.horizontal_fov = 1.3962634; // rad
    cam_params.vertical_fov = 2 * atan(tan(cam_params.horizontal_fov / 2) * 1);
    cam_params.image_width = 800;
    cam_params.image_height = 800;
    cam_params.near_clip = 0.1;
    cam_params.near_clip = 0.02;
    cam_params.far_clip = 300;
    cam_params.noise_mean = 0.0;
    cam_params.noise_std_dev = 0.007;
    cam_params.hack_baseline = 0.07;
    cam_params.distortion_k1 = 0.0;
    cam_params.distortion_k2 = 0.0;
    cam_params.distortion_k3 = 0.0;
    cam_params.distortion_t1 = 0.0;
    cam_params.distortion_t2 = 0.0;
    cam_params.camera_pos_x = 75; // cm
    cam_params.camera_pos_y = 0;
    cam_params.camera_pos_z = 202.5;
    cam_params.principal_point_x = 400.5;
    cam_params.principal_point_y = 400.5;
    cam_params.translation_x = -33.36921585209936;
    cam_params.translation_y = 0;
    cam_params.vanishing_point_x = INPUT_HEIGHT / 2;
    cam_params.vanishing_point_y = INPUT_WIDTH / 2;
}

void ScanLines(vector<Point> &dst, Mat &frame_src)
{
    int center_x = frame_src.cols / 2;
    int center_y = frame_src.rows;

    int start_angle = 0; // Starting angle in degrees
    int end_angle = 180; // Ending angle in degrees
    int num_angles = end_angle - start_angle + 1;

    dst.resize(num_angles);

    for (int16_t i = start_angle; i < end_angle; i++)
    {
        float pixel = 100;
        float px_x = center_x;
        float px_y = center_y;

        px_x = px_x + pixel * cos(i * M_PI / 180 * center_y);
        px_y = px_y + pixel * sin(i * M_PI / 180 * center_y);

        while (px_x >= 0 && px_y >= 0 && px_x < frame_src.cols && px_y < frame_src.rows)
        {
            line(frame_src, Point(px_x, px_y), Point(px_x, px_y), Scalar(255, 255, 255), 1);
            if (frame_src.at<uchar>(px_y, px_x) == 255)
            {
                printf("Found line at %d degrees\n", i);
                dst[i] = Point(px_x, px_y);
                break;
            }
            pixel++;
            px_x = px_x + (pixel * cos(i * M_PI / 180));
            px_y = px_y + (pixel * sin(i * M_PI / 180));
        }
        if (px_x < 0 || px_y < 0 || px_x >= frame_src.cols || px_y >= frame_src.rows)
        {
            dst[i] = Point(-1, -1);
        }
    }

    // for (int16_t i = start_angle; i < end_angle; i++)
    // {
    //     float angle = static_cast<float>(i);
    //     float angle_in_rad = DEG2RAD(angle);

    //     int x = center_x + static_cast<int>(cos(angle_in_rad) * center_y);
    //     int y = center_y - static_cast<int>(sin(angle_in_rad) * center_y);

    //     // Check if the point is within the frame boundaries
    //     if (x >= 0 && x < frame_src.cols && y >= 0 && y < frame_src.rows)
    //     {
    //         printf("%f\n", frame_src.at<unsigned char>(Point(x, y)));
    //         if (frame_src.at<float>(x, y) == 250)
    //         {
    //             printf("Found at %d\n", i);
    //             dst[i - start_angle].x = x;
    //             dst[i - start_angle].y = y;
    //         }
    //     }
    //     else
    //     {
    //         dst[i - start_angle].x = -1;
    //         dst[i - start_angle].y = -1;
    //     }
    //     // line(frame_src, Point(center_x, center_y), Point(x, y), Scalar(255, 255, 255), 1);
    // }
}

void BuildIPMTable(const int src_w, const int src_h, const int dst_w, const int dst_h, const int vanishing_pt_x, const int vanishing_pt_y, int *maptable)
{
    // float alpha = cam_params.horizontal_fov / 2;
    // float gamma = -(float)(vanishing_pt_x - (src_w >> 1)) * alpha / (src_w >> 1);
    // float theta = -(float)(vanishing_pt_y - (src_h >> 1)) * alpha / (src_h >> 1);

    float alpha = 0.686111;
    float gamma = 0;
    float theta = 0.069444;

    int front_map_pose_start = (dst_h >> 1);
    int front_map_pose_end = front_map_pose_start + dst_h + 200;

    int side_map_mid_pose = dst_w >> 1;

    // int front_map_scale = cam_params.cam_scale_y;
    // int side_map_scale = cam_params.cam_scale_x;

    int front_map_scale = 2;
    int side_map_scale = 0;

    for (int y = 0; y < dst_w; ++y)
    {
        for (int x = front_map_pose_start; x < front_map_pose_end; ++x)
        {
            int index = y * dst_h + (x - front_map_pose_start);

            int delta_x = front_map_scale * (front_map_pose_end - x - cam_params.camera_pos_x);
            int delta_y = front_map_scale * (y - side_map_mid_pose - cam_params.camera_pos_y);

            if (!delta_y)
                maptable[index] = maptable[index - dst_h];
            else
            {
                int u = (int)((atan(cam_params.camera_pos_z * sin(atan((float)delta_y / delta_x)) / delta_y) - (theta - alpha)) / (2 * alpha / src_h));
                int v = (int)((atan((float)delta_y / delta_x) - (gamma - alpha)) / (2 * alpha / src_w));

                if (u >= 0 && u < src_h && v >= 0 && v < src_w)
                    maptable[index] = src_w * u + v;
                else
                    maptable[index] = -1;
            }
        }
    }
}

void InversePerspective(const int dst_w, const int dst_h, const unsigned char *src, const int *maptable, unsigned char *dst)
{
    int index = 0;
    for (int j = 0; j < dst_h; ++j)
    {
        for (int i = 0; i < dst_w; ++i)
        {
            if (maptable[index] == -1)
            {
                dst[i * dst_h + j] = 0;
            }
            else
            {
                dst[i * dst_h + j] = src[maptable[index]];
            }
            ++index;
        }
    }
}

void InversePerspective1(int dst_w, int dst_h, int *dst, int src_w, int src_h, int *src)
{
    int i, j;
    int x, y, z;
    int u, v;
    int *p;
    int *q;

    for (i = 0; i < dst_h; i++)
    {
        for (j = 0; j < dst_w; j++)
        {
            x = dst[i * dst_w + j * 3 + 0];
            y = dst[i * dst_w + j * 3 + 1];
            z = dst[i * dst_w + j * 3 + 2];

            u = maptable[i * dst_w + j * 3 + 0];
            v = maptable[i * dst_w + j * 3 + 1];

            if (u < 0 || u >= src_w || v < 0 || v >= src_h)
            {
                src[i * dst_w + j * 3 + 0] = 0;
                src[i * dst_w + j * 3 + 1] = 0;
                src[i * dst_w + j * 3 + 2] = 0;
            }
            else
            {
                p = &src[v * src_w + u * 3];
                q = &src[i * dst_w + j * 3];
                q[0] = p[0];
                q[1] = p[1];
                q[2] = p[2];
            }
        }
    }
}

Mat ConvertToImage(int *src)
{
    Mat image(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, Scalar(0, 0, 0));
    int i, j;
    int *p;
    uchar *q;

    for (i = 0; i < INPUT_HEIGHT; i++)
    {
        for (j = 0; j < INPUT_WIDTH; j++)
        {
            p = &src[i * INPUT_WIDTH + j];
            q = &image.at<Vec3b>(i, j)[0];
            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }
    return image;
}

void DrawVanishingPoint(Mat *frame)
{
    line(*frame, Point(0, 400), Point(800, 400), Scalar(0, 0, 255));
    line(*frame, Point(400, 0), Point(400, 800), Scalar(0, 0, 255));
}