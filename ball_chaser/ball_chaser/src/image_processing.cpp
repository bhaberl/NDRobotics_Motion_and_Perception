#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "ball_chaser/NormalizedPosition.h"

#include <string>

double GetNormalizedComponent(int index, int num_pixels_chosen_dim, bool is_order_reversed = false)
{
    int midpoint = num_pixels_chosen_dim / 2;
    // Correction to get symmetric values for even number of pixels\
    // in a row/column. In this case, the result will +-k/(midpoint-1),
    // for k = 0, ..., midpoint-1, mapping the two central columns to 0
    if(num_pixels_chosen_dim % 2 == 0)
    {
        --midpoint;
        if(index > midpoint)
            --index;
    }
    double norm_comp = static_cast<double>(index - midpoint)/static_cast<double>(midpoint);
    return is_order_reversed ? -norm_comp : norm_comp;
}


// Processes image and publishes normalized position
// of first white pixel found, which is the utmost left
// white in the lowest row of pixels containing white
// pixels.
// Values of horizontal/vertical component of the
// normalized position range from -1 to 1, with 0
// corresponding to the middle column/row (or pair of
// columns/rows if the number of pixels per row/column is even).
// In the case the bool in published message is set to false
// (there was no white pixel), normalized position should
// be ignored.
// Message type is ball_chaser::NormalizedPosition
// Topic it is published into is "/ball_chaser/ball_norm_position"
class ImageProcessor
{
    ros::Publisher location_publisher_;
    void ProcessImageCallback(const sensor_msgs::Image& img);

public:
    ImageProcessor() {}
    // Run the image processor
    void Run();


};

void ImageProcessor::Run()
{
    ros::NodeHandle nodeh;

    // Subscribe to /camera/rgb/image_raw topic to read the image data
    ros::Subscriber sub_image = nodeh.subscribe("/camera/rgb/image_raw", 10,
                                                &ImageProcessor::ProcessImageCallback,
                                                this);

    // Inform ROS master that we will be publishing a message of type
    // ball_chaser::NormalizedPosition on the /ball_chaser/ball_norm_position
    // topic with a publishing queue size of 10
    location_publisher_ =
        nodeh.advertise<ball_chaser::NormalizedPosition>("/ball_chaser/ball_norm_position", 10);

    // Handle ROS communication events
    ros::spin();
}

void ImageProcessor::ProcessImageCallback(const sensor_msgs::Image& img)
{
    ball_chaser::NormalizedPosition pos;

    uint8_t saturated_color_component{255};
    bool is_there_white_pixel{false};
    double hor_pos_white_pixel{0.0};
    double ver_pos_white_pixel{0.0};
    int red_byte_offset = 0;
    int green_byte_offset = 1;
    int blue_byte_offset = 2;
    for(int row = img.height - 1; row >= 0; --row)
    {
        for(int column = 0; column < img.step; column+=3)
        {
            int index_raw_data = row * img.step + column;
            if ( img.data[index_raw_data + red_byte_offset] == saturated_color_component
              && img.data[index_raw_data + green_byte_offset] == saturated_color_component
              && img.data[index_raw_data + blue_byte_offset] == saturated_color_component)
            {
                is_there_white_pixel = true;
                const bool is_order_reversed_vertical = true;
                hor_pos_white_pixel = GetNormalizedComponent(column, img.step);
                ver_pos_white_pixel = GetNormalizedComponent(row, img.height, is_order_reversed_vertical);
                break;
            }
        }
        if (is_there_white_pixel)
            break;
    }

    // In the case there is no white pixel, normalized position
    // will be ignored, so we can fill it anyway.
    pos.contains_object = is_there_white_pixel;
    pos.horizontal = hor_pos_white_pixel;
    pos.vertical = ver_pos_white_pixel;

    // Publish normalized position
    location_publisher_.publish(pos);
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "process_image");

    // Initialize image processor
    ImageProcessor processor;

    // Run processor
    processor.Run();

    return 0;
}