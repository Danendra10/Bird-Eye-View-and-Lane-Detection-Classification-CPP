#include <opencv2/opencv.hpp>

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

int main()
{
    std::string baseDirectory = "../assets/generate_images/";
    int numDirectories = 8;

    cv::Mat inputImage = cv::imread("../assets/first_frame.jpg");
    for (int i = 1; i <= numDirectories; ++i)
    {
        std::string arucoPath = baseDirectory + std::to_string(i) + "/marker" + std::to_string(i) + ".png";
        std::string outputPath = baseDirectory + std::to_string(i) + "/output_image.jpg";

        // Load the input_image and the aruco image
        cv::Mat arucoImage = cv::imread(arucoPath);

        // Define the target size for the aruco image
        cv::Size targetSize(40, 39);
        // cv::Size targetSize(arucoImage.cols, arucoImage.rows);

        // Resize the aruco image to fit the target size
        cv::resize(arucoImage, arucoImage, targetSize);

        // Define the coordinates for placing the aruco image
        cv::Rect region(cv::Point(570, 296), cv::Point(570 + targetSize.width, 296 + targetSize.height));

        // Copy the resized aruco image onto the input_image at the specified region
        arucoImage.copyTo(inputImage(region));

        // Save the modified input_image
        cv::imwrite(outputPath, inputImage);
    }

    return 0;
}
