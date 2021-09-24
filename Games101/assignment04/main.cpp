#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 5)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> temp_control_points1;
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        auto p = t * control_points[i] + (1 - t) * control_points[i + 1];
        temp_control_points1.emplace_back(p);
    }
    if (temp_control_points1.size() == 1)
    {
        return temp_control_points1[0];
    }
    return recursive_bezier(temp_control_points1, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += 0.0001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        //* 提高
        float minX = std::floor(point.x);
        float minY = std::floor(point.y);
        float fract_x = point.x - minX;
        float fract_y = point.y - minY;
        int x_flag = fract_x < 0.5f ? -1 : 1;
        int y_flag = fract_y < 0.5f ? -1 : 1;

        cv::Point2f p00 = cv::Point2f(minX + 0.5f, minY + 0.5f);
        cv::Point2f p01 = cv::Point2f(minX + x_flag + 0.5f, minY + 0.5f);
        cv::Point2f p10 = cv::Point2f(minX + 0.5f, minY + y_flag + 0.5f);
        cv::Point2f p11 = cv::Point2f(minX + x_flag + 0.5f, minY + y_flag + 0.5f);

        std::vector<cv::Point2f> vec;
        vec.push_back(p01);
        vec.push_back(p10);
        vec.push_back(p11);

        float dis1 = cv::norm(p00 - point);

        for (auto p : vec)
        {
            float dis = cv::norm(p - point);
            float color = window.at<cv::Vec3b>(p.y, p.x)[1];
            window.at<cv::Vec3b>(p.y, p.x)[1] = std::max(color, 255 * dis1 / dis);
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);
    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 5)
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
