#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string videoPath;
    string imagePath = "../assets/output_image.jpg";
    // string imagePath = "../assets/generate_images/1/output_image.jpg";

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];

        if (arg.substr(0, 2) == "v=")
        {
            videoPath = arg.substr(2);
            imagePath = "";
        }
        else if (arg.substr(0, 2) == "i=")
        {
            imagePath = arg.substr(2);
            videoPath = "";
        }
    }

    if (!videoPath.empty())
    {
        cout << "Processing video: " << videoPath << endl;
    }
    else if (!imagePath.empty())
    {
        cout << "Processing image: " << imagePath << endl;
        Mat input_image = imread(imagePath);

        cvtColor(input_image, input_image, COLOR_BGR2GRAY);

        // threshold(input_image, input_image, 100, 255, THRESH_BINARY);
        // erode(input_image, input_image, Mat(), Point(-1, -1), 2);
        // dilate(input_image, input_image, Mat(), Point(-1, -1), 2);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(input_image, markerCorners, markerIds, rejectedCandidates);

        cv::Mat outputImage = input_image.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        printf("Detected %lu markers\n", markerIds.size());

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
