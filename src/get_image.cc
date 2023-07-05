#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Please provide the video path as an argument." << std::endl;
        return 1;
    }

    // Read the video path from the command-line argument
    std::string vide_path = argv[1];

    // Open the video file
    VideoCapture cap(vide_path);

    if (!cap.isOpened())
    {
        std::cout << "Failed to open the video file." << std::endl;
        return 1;
    }

    // Read the video frames
    Mat frame;
    while (cap.read(frame))
    {
        // save only first frame to file
        imwrite("first_frame.jpg", frame);
        break;
    }

    // Release the video capture object
    cap.release();

    return 0;
}
