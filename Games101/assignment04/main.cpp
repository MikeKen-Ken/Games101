#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 15)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x
                  << ", " << 700 - y << ")" << '\n';
        control_points.emplace_back(x, 700 - y);
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
        auto point = std::pow(1 - t, 3) * p_0 +
                     3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(700 - point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points,
                             float t)
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

//* p to segment p1-p2 distance
float distance(cv::Point2f p, cv::Point2f p1, cv::Point2f p2)
{
    cv::Vec2f v1(p - p1);
    cv::Vec2f v2(p2 - p1);
    float dis1 = cv::norm(p - p1);
    float dis2 = cv::norm(p2 - p1);
    return (v1.dot(v2) / (dis1 * dis2)) * dis1;
}

void drawPoint(cv::Point2f point, Eigen::Vector3f line_color, cv::Mat &window)
{
    //* 首先，p点本身需要上色
    window.at<cv::Vec3b>(700 - point.y, point.x)[1] = 255;

    //* 提高 反走样
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
        //* 根据距离来计算，像素点的颜色
        float dis = cv::norm(p - point);
        float color = window.at<cv::Vec3b>(700 - p.y, p.x)[1];
        //* 如果所在点已经有颜色，则相比之下取最大值
        window.at<cv::Vec3b>(700 - p.y, p.x)[1] =
            std::max(color, 255 * dis1 / dis);
    }
}

void draw_line(cv::Point2f begin, cv::Point2f end, cv::Mat &window)
{
    auto x1 = begin.x;
    auto y1 = begin.y;
    auto x2 = end.x;
    auto y2 = end.y;

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }
        cv::Point2f point = cv::Point2f(x, y);
        drawPoint(point, line_color, window);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            cv::Point2f point = cv::Point2f(x, y);
            drawPoint(point, line_color, window);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }
        cv::Point2f point = cv::Point2f(x, y);
        drawPoint(point, line_color, window);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            cv::Point2f point = cv::Point2f(x, y);
            drawPoint(point, line_color, window);
        }
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de
    // Casteljau's recursive Bezier algorithm.
    std::vector<cv::Point2f> allPoints;
    for (double t = 0.0; t <= 1.0; t += 0.0001)
    {
        auto point = recursive_bezier(control_points, t);
        allPoints.push_back(point);
    }

    //* Use Ramer-Douglas-Peucker algorithm simplify the points
    const float epsilon = 30;

    std::vector<cv::Point2f> simplifyPoints;
    simplifyPoints.push_back(allPoints[0]);
    int curIndex = 0;
    int length = allPoints.size();
    for (int i = 1; i < length; i++)
    {
        float dis =
            distance(allPoints[i], allPoints[curIndex], allPoints[length - 1]);
        if (dis > epsilon)
        {
            curIndex = i;
            simplifyPoints.push_back(allPoints[i]);
        }
    }
    simplifyPoints.push_back(allPoints[length - 1]);

    for (int i = 0; i < simplifyPoints.size() - 1; i++)
    {
        draw_line(simplifyPoints[i], simplifyPoints[i + 1], window);
        //* Or draw the points
        // Eigen::Vector3f line_color = {255, 255, 255};
        // drawPoint(simplifyPoints[i], line_color, window);
    }
}

int findTheBottomPointIndex(const std::vector<cv::Point2f> &control_points)
{
    int bottomIndex = 0;
    for (int i = 1; i < control_points.size(); i++)
    {
        cv::Point2f curPoint = control_points[i];
        cv::Point2f botPoint = control_points[bottomIndex];
        if (curPoint.y < botPoint.y ||
            (curPoint.y == botPoint.y && curPoint.x < botPoint.x))
        {
            bottomIndex = i;
        }
    }
    return bottomIndex;
}

struct PointWithCosine
{
    float cosine;
    cv::Point2f point;
};

bool compareCosine(PointWithCosine i1, PointWithCosine i2)
{
    return (i1.cosine > i2.cosine);
}

enum ConvexHullAlgorithm
{
    Graham_Scan,
    Chan
};

std::vector<cv::Point2f> getPointsByGrahamScan(
    const std::vector<cv::Point2f> &control_points)
{
    //* Graham's Scan algorithm
    std::vector<PointWithCosine> sortedPonts;
    int botIndex = findTheBottomPointIndex(control_points);
    cv::Point2f botPoint = control_points[botIndex];
    cv::Point2f xAxis(1, 0);
    for (int i = 0; i < control_points.size(); i++)
    {
        if (i == botIndex)
        {
            continue;
        }
        float cosine = (control_points[i] - botPoint).dot(xAxis) /
                       cv::norm(control_points[i] - botPoint);
        sortedPonts.push_back({cosine, control_points[i]});
    }
    std::sort(sortedPonts.begin(), sortedPonts.end(), compareCosine);

    std::vector<cv::Point2f> resultPoints;
    //* push start point
    resultPoints.push_back(botPoint);
    resultPoints.push_back(sortedPonts[0].point);
    for (int i = 1; i < sortedPonts.size(); i++)
    {
        float cosine = (resultPoints[resultPoints.size() - 1] -
                        resultPoints[resultPoints.size() - 2])
                           .cross(sortedPonts[i].point -
                                  resultPoints[resultPoints.size() - 1]);
        if (cosine > 0)
        {
            resultPoints.push_back(sortedPonts[i].point);
        }
        else
        {
            resultPoints.pop_back();
            i--;
        }
    }
    //* push end point
    resultPoints.push_back(botPoint);
    return resultPoints;
}

findNextPointByTangent(onst std::vector<cv::Point2f> &control_points,
                       cv::Point2f lastPoint)
{
    cv::Point2f lowestPonit;
}

void getConvexHull(const std::vector<cv::Point2f> &control_points,
                   cv::Mat &window, ConvexHullAlgorithm type)
{
    std::vector<cv::Point2f> resultPoints;
    switch (type)
    {
    case Graham_Scan: {
        resultPoints = getPointsByGrahamScan(control_points);
    }
    case Chan: {
        int pointsCount = control_points.size();
        int botIndex = findTheBottomPointIndex(control_points);
        cv::Point2f botPoint = control_points[botIndex];

        for (int t = 1; t < pointsCount; t++)
        {
            resultPoints.clear();
            int m = std::min(pointsCount, (int)std::pow(2, std::pow(2, t)));
            int k = std::ceil((float)pointsCount / (float)m); // m = 4 k = 4
            std::vector<std::vector<cv::Point2f>> subhulls;

            for (int i = 0; i < k; i++)
            {
                std::vector<cv::Point2f> subPoints;
                for (int j = 0; j < m; j++)
                {
                    if (j + i * m >= pointsCount)
                    {
                        break;
                    }
                    subPoints.push_back(control_points[j + i * m]);
                }
                subhulls.push_back(getPointsByGrahamScan(subPoints));
            }
            // find p0 with each subhull tangent , find all the lowest p1
            resultPoints.push_back(botPoint);
            resultPoints.push_back(
                findNextPointByTangent(resultPoints.size() - 1, subhulls));
            if (resultPoints[resultPoints.size() - 1] == botPoint)
            {
                break;
            }
        }
    }
    }
    for (int i = 0; i < resultPoints.size() - 1; i++)
    {
        draw_line(resultPoints[i], resultPoints[i + 1], window);
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
            cv::circle(window, cv::Point2f(point.x, 700 - point.y), 3,
                       {255, 0, 255}, 3);
        }

        if (control_points.size() == 15)
        {
            // naive_bezier(control_points, window);
            // bezier(control_points, window);
            getConvexHull(control_points, window, ConvexHullAlgorithm::Chan);

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
