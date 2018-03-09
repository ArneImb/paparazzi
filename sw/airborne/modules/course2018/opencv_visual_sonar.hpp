#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

extern uint16_t number_positives_square(Mat integral_img, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom);
extern uint16_t pixels_to_go(Mat mask, uint8_t square_width, uint8_t square_height, float threshold);
extern float pix_to_m(uint16_t pixels);
