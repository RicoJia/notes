#include <opencv2/opencv.hpp>

using namespace cv;

int main(){

Mat image;
image = imread("sample.jpeg");
namedWindow("Display Image", WINDOW_AUTOSIZE);
imshow("Display Image", image);
waitKey(0);

return 0;
}
