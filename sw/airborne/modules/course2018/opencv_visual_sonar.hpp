#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

uint16_t number_positives_square(Mat integral_img, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom);
uint16_t pixels_to_go(Mat mask, int square_size, float threshold);
