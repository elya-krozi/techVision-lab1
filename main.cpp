#include "iostream"
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static const int POINT_NUMBER = 100;
static const int SIN_FREQUENCY_INCREASER = 4;
static const int HORIZONTAL_MARGIN_START = 110;
static const int SIZE_ROBOT_REDUCER = 4;

int main() {
    Mat robo_path = imread("sources/background/floor.JPG");
    Mat robo_path_sin = robo_path.clone();
    Mat robo_path_robot = robo_path.clone();
    Mat robo_path_final = robo_path.clone();

    Mat robot = imread("sources/robot/robot.JPG");
    resize(robot,
           robot, 
           Size(lround(robot.cols / SIZE_ROBOT_REDUCER), lround(robot.rows / SIZE_ROBOT_REDUCER)),
           INTER_AREA);
    robot.convertTo(robot, -1, 1.05, 0.7);

    int vertical_robot_position[POINT_NUMBER] = {0};

    int horizontal_position_start = HORIZONTAL_MARGIN_START;
    int horizontal_position_end = robo_path_sin.cols - robot.cols;
    int horizontal_step = lround((horizontal_position_end - horizontal_position_start) / POINT_NUMBER);
    int sin_amplitude = lround((robo_path.rows - robot.rows) / 2);

    for (int step_number = 0; step_number < POINT_NUMBER; step_number++)
    {
        vertical_robot_position[step_number] = lround(robo_path_sin.rows / 2 +
                                                        sin_amplitude * sin (
                                                                        (step_number * CV_PI * SIN_FREQUENCY_INCREASER) /
                                                                            (POINT_NUMBER)
                                                                        )
                                                 );
    }

    imshow("robo_path", robo_path);
    waitKey();

    Mat robot_gray;
    Mat robot_thresholded(robot.rows, robot.cols, 5);
    Mat robot_bitwised_or;
    Mat robot_thresholded_not;
    Mat robot_fr;
    Mat roi_final;

    while (waitKey() != 27)
    {
        int horizontal_image_position = horizontal_position_start;

        for (int step_number = 0; step_number < POINT_NUMBER; step_number++)
        {
            Rect roi(Point(horizontal_image_position - HORIZONTAL_MARGIN_START, vertical_robot_position[step_number] - robot.rows / 2), Size(robot.cols, robot.rows));
            Mat robo_path_robot_roi = robo_path_final(roi);

            cvtColor(robot, robot_gray, COLOR_RGB2GRAY);

            threshold(robot_gray, robot_thresholded , 254,  255,  THRESH_BINARY);

            bitwise_or(robo_path_robot_roi, robo_path_robot_roi, robot_bitwised_or, robot_thresholded);

            bitwise_not(robot_gray, robot_thresholded_not);

            bitwise_and(robot, robot, robot_fr, robot_thresholded_not);

            add(robot_bitwised_or, robot_fr, roi_final);

            robo_path_final = robo_path_sin.clone();

            roi_final.copyTo(robo_path_final(roi));

            line(robo_path_sin,
                 Point(horizontal_image_position, vertical_robot_position[step_number]),
                 Point(horizontal_image_position + horizontal_step, vertical_robot_position[step_number + 1]),
                 CV_RGB(255, 255, 255),
                 5,
                 LINE_8,
                 0);

            imshow("robo_path", robo_path_final);
            waitKey(7);

            if (horizontal_image_position > (lround(horizontal_position_end / 2) - horizontal_step) &&
               horizontal_image_position < (lround(horizontal_position_end / 2) + horizontal_step))
            {
                imwrite("sources/result/middleRoboPath.jpg", robo_path_final);
            }

            robo_path_final = robo_path_sin.clone();

            horizontal_image_position += horizontal_step;
        }

        robo_path_sin = robo_path.clone();
        robo_path_final = robo_path.clone();
    }

    return 0;
}