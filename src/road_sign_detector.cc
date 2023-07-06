#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
/**
 * 1 dead end
 * 2 end tunnel
 * 3. forward
 * 4. left
 * 5. no entry
 * 6. right
 * 7. start tunnel
 * 8. stop
 */

const string commands[] = {"dead end", "end tunnel", "forward", "left", "no entry", "right", "start tunnel", "stop"};

int main(int argc, char **argv)
{
    string video_path;
    string image_path = "../assets/output_image.jpg";
    image_path = "../assets/generate_images/1/output_image.jpg";

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];

        if (arg.substr(0, 2) == "v=")
        {
            video_path = arg.substr(2);
            image_path = "";
        }
        else if (arg.substr(0, 2) == "i=")
        {
            image_path = arg.substr(2);
            video_path = "";
        }
    }

    if (!video_path.empty())
    {
        cout << "Processing video: " << video_path << endl;
    }
    else if (!image_path.empty())
    {
        cout << "Processing image: " << image_path << endl;
        Mat input_image = imread(image_path);

        // cvtColor(input_image, input_image, COLOR_BGR2GRAY);

        // threshold(input_image, input_image, 100, 255, THRESH_BINARY);
        // erode(input_image, input_image, Mat(), Point(-1, -1), 2);
        // dilate(input_image, input_image, Mat(), Point(-1, -1), 2);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(input_image, markerCorners, markerIds, rejectedCandidates);

        cv::Mat outputImage = input_image.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        printf("Detected %lu markers\n", markerIds.size());

        for (int i = 0; i < markerIds.size(); ++i)
        {
            int id = markerIds[i];
            cv::Point2f center = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) / 4;
            cv::putText(outputImage, commands[id - 1], center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        imshow("Input Image", input_image);
        imshow("output", outputImage);
        waitKey(0);
    }
    else
    {
        cout << "Invalid arguments." << endl;
        return 1;
    }

    return 0;
}
