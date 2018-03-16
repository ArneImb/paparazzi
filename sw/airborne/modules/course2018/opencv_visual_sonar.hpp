#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

extern uint16_t func_square_height(uint16_t pos, uint16_t square_height_min, uint16_t square_height_max);
extern uint16_t number_positives_square(Mat integral_img, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom);
extern uint16_t pixels_to_go(Mat mask, uint8_t square_width, uint16_t square_height_min, uint16_t square_height_max, float threshold);
extern float pix_to_m(uint16_t pixels);

extern float aggression;
extern uint16_t focal; 																													//focal distance camera in pixels
extern uint8_t screen_height;
