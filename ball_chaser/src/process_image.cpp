#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


namespace process_image
{
    // Generic, endianness agnostic, memory representation of a pixel
    template <typename _T_CHANNEL, size_t _COUNT>
    struct pixel
    {
        _T_CHANNEL channels[_COUNT];

        bool operator==(const pixel& rhs) const
        {
            return std::equal(std::begin(channels), std::end(channels), rhs.channels);
        }

        bool operator!=(const pixel& rhs) const
        {
            return !std::equal(std::begin(channels), std::end(channels), rhs.channels);
        }
    };

    // RGB8 encoded pixel alias
    using rgb8_pixel = pixel<std::uint8_t, 3>;

    // Pixel iterator alias
    using const_rgb8_pixel_iterator = const rgb8_pixel *;

    // White RGB8 pixel constant value
    constexpr rgb8_pixel White_rgb8_pixel = { {255,255,255} };
    
    // Max velocities constants
    constexpr float max_linear_x_abs = 0.5f;
    constexpr float max_angular_z_abs = 0.5f;

    // Performs a sequential lookup for a white rgb8_pixel in a range of rgb8_pixel elements.
    const_rgb8_pixel_iterator find_white_pixels(const_rgb8_pixel_iterator begin, const_rgb8_pixel_iterator end)
    {
        // search for a white pixel in pixel range
        auto it = std::find(begin, end, White_rgb8_pixel);

        if (it != end)
        {
            // advance iterator to approximate center of white pixels group
            auto last = std::find_if(it, end, [](const rgb8_pixel& p) {return p != White_rgb8_pixel; });
            auto distance = std::distance(it, --last);

            if (distance > 2) // only advance if last white pixel is more than 2 pixels away
                std::advance(it, std::distance(it, --last) / 2);
        }

        return it;
    }
    
    // Finds the middle of a white rgb8_pixel sequence given a range of rgb8_pixels begining with a white rgb8_pixel.
    const_rgb8_pixel_iterator find_middle_of_white_pixels(const_rgb8_pixel_iterator begin, const_rgb8_pixel_iterator end)
    {
        // Find white pixel sequence end
        auto it = std::find_if(begin, end, [](const rgb8_pixel& p) {return p != White_rgb8_pixel; });        

        // Advance to approximate center of white pixels sequence (only if there are more than 2 consecutive white pixels)
        auto distance = std::distance(begin, --it);
        if (distance > 2) 
            std::advance(it, distance / 2);

        return it;
    }

    // Looks for a white ball in the camera image and returns absolute pixel offset
    bool get_white_ball_pixel_offset(const sensor_msgs::Image img, size_t& offset)
    {
        // White ball cannot be in the lower half of the image, camera perspective makes it impossible, so we only scan the upper half.
        // As an optimization, we first scan in the horizontal pixel line in the middle of the image, where it is more likely to be found.
        // If we don't find it, we scan the upper half of the image in case the ball is in the air or on top of some object.        
        
        // Get middle pixel row offset (store in a static variable so we only do it once)
        static const size_t middle_pixel_row_offset = (img.height / 2) * img.width;

        // Get a pointer to image data as rgb8_pixel elements
        auto first_pixel = reinterpret_cast<const_rgb8_pixel_iterator>(img.data.data());

        // Look for a white pixel in the middle row of pixels
        auto begin = std::next(first_pixel, middle_pixel_row_offset);
        auto end = std::next(begin, img.width);

        auto it = std::find(begin, end, White_rgb8_pixel);

        if (it != end)
        {
            // get the offset to the middle of white pixels sequence
            offset = std::distance(first_pixel, find_middle_of_white_pixels(it, end));
            return true;
        }

        // If no luck searching in middle row, look for target in image upper half
        end = begin; // begin points to first pixel of middle row, that is, upper half end
        begin = first_pixel;
        
        it = std::find(begin, end, White_rgb8_pixel);

        if (it != end)
        {
            // get the offset to the middle of white pixels sequence
            offset = std::distance(first_pixel, find_middle_of_white_pixels(it, end));
            return true;
        }

        return false;
    }

    // Calculates linear_x and angular_z values from image.
    // Looks for the white ball and calculates angular_z and linear_x based on its position.
    void calculate_velocities_from_image(const sensor_msgs::Image img, float& linear_x, float& angular_z)
    {
        // Initialize linear_x and angular_z to default value 0.0 to make robot stop in case we don't find the white ball
        linear_x = 0.0f;
        angular_z = 0.0f;

        // Search for white ball
        size_t white_ball_pixel_offset = 0;

        if (get_white_ball_pixel_offset(img, white_ball_pixel_offset))
        {
            // We calculate an angular_z value proportional to the deviation from the center of the image, in the range {-0.5, 0.5}.
            // Then we calculate a linear_x value inversely proportional to angular_z absolute value, in the range {0, 0.5}.
        
            static const size_t center_horizontal_offset = img.width / 2;

            angular_z = (((float)center_horizontal_offset - (float)(white_ball_pixel_offset % img.width)) * max_angular_z_abs) / (float)center_horizontal_offset;
            linear_x = max_linear_x_abs - std::abs(angular_z);
        }
    }

    // Process image node implementation
    class Node
    {
        // ROS::NodeHandle
        ros::NodeHandle node = ros::NodeHandle();

        // ROS::ServiceClient to command robot
        ros::ServiceClient command_robot_svc_client = node.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // ROS::Subscriber to receive camera images
        ros::Subscriber camera_image_subscriber = node.subscribe("/camera/rgb/image_raw", 10, &Node::process_image_callback, this);

        // This function calls the command_robot service to drive the robot in the specified direction
        void drive_robot(float lin_x, float ang_z)
        {
            // Request a service and pass the velocities to it to drive the robot
            ball_chaser::DriveToTarget srv;
            srv.request.linear_x = lin_x;
            srv.request.angular_z = ang_z;

            if (!command_robot_svc_client.call(srv))
                ROS_ERROR("Failed to call service command_robot");
        }

        // This callback function continuously executes and reads the image data
        void process_image_callback(const sensor_msgs::Image img)
        {            
            float linear_x = 0.0f;
            float angular_z = 0.0f;

            calculate_velocities_from_image(img, linear_x, angular_z);
            drive_robot(linear_x, angular_z);
        }
    };
}

template <typename _NodeImpl>
void RunNodeSync(int argc, char** argv, const char* node_name)
{
    ros::init(argc, argv, node_name);
    _NodeImpl node;
    ros::spin();
}


int main(int argc, char** argv)
{
    ROS_INFO_STREAM("main()");
    
    // Run process_image node
    RunNodeSync<process_image::Node>(argc, argv, "process_image");
    return 0;
}
