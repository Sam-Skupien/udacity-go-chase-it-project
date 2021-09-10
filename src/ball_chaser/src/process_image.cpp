#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"


ros::ServiceClient client;

enum Direction {stay, left, center, right};

Direction getDriveDirection(const int left, const int center, const int right) {
    // not white pixels were found
    if (!left && !center && !right) {
        return Direction::stay;
    }
    
    if(left > center && left > right) {
        return Direction::left;
    }

    if(center > left && center > right) {
        return Direction::center;
    }

    if(right > center && right > left) {
        return Direction::right;
    }

    return Direction::stay;
}

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);

    ROS_INFO_STREAM("Service Response: " << srv.response.msg_feedback);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    int partition = img.step/3;

    // create counters to get pixel density in each zone
    int left = 0;
    int center = 0;
    int right = 0;

    //ROS_INFO_STREAM("Width (columns): " << img.width << " Height (rows): " << img.height << "\n\n");
    

    for(int row = 0; row < img.height; row++) {
        for(int colByte = 0; colByte < img.step; colByte+=3) {
            //ROS_INFO_STREAM("row: " << row <<  " colByte " << colByte << "\n");
            if(img.data[row * img.step + colByte] == white_pixel
               && img.data[row * img.step + colByte + 1 == white_pixel]
               && img.data[row * img.step + colByte + 2] == white_pixel) {
                   //ROS_INFO_STREAM("row: " << row <<  " colByte " << colByte << "\n");
                   if(colByte < partition) {
                       left++;
                   } else if (colByte < partition * 2) {
                       center++;
                   } else {
                       right++;
                   }
               }
            //ROS_INFO_STREAM("Byte R data[i]: " << static_cast<unsigned int>(img.data[row * img.step + colByte]) << "\n");
            //ROS_INFO_STREAM("Byte G data[i + 1]: " << static_cast<unsigned int>(img.data[row * img.step + colByte + 1]) << "\n");
            //ROS_INFO_STREAM("Byte B data[i + 2]: " << static_cast<unsigned int>(img.data[row * img.step + colByte + 2]) << "\n\n");
        }
    }

    //ROS_INFO_STREAM("left: " << left <<  " center: " << center << " right: " << right << "\n");
    
    Direction driveDirection = getDriveDirection(left, center, right);

    switch(driveDirection) {
        case Direction::stay:
        ROS_INFO_STREAM("Staying\n");
        drive_robot(0.0, 0.0);
        break;

        case Direction::left:
        ROS_INFO_STREAM("Driving Left\n");
        drive_robot(0.0, -0.5);
        break;

        case Direction::center:
        ROS_INFO_STREAM("Driving Center\n");
        drive_robot(0.5, 0.0);
        break;

        case Direction::right:
        ROS_INFO_STREAM("Driving Right\n");
        drive_robot(0.0, 0.5);
        break;

        default:
        ROS_INFO_STREAM("Default\n");
        drive_robot(0.0, 0.0);
    }

}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
