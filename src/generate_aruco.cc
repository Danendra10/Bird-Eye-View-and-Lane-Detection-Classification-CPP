#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

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
    for (int i = 1; i <= 8; i++)
    {
        Mat marker_image;
        aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        aruco::generateImageMarker(dictionary, i, 200, marker_image, 1);
        string file_name = "marker" + to_string(i) + ".png";
        imwrite(file_name, marker_image);
    }
}