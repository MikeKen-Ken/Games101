#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // TODO: Implement this function
    // TODO: Create the model matrix for rotating the triangle around the Z axis.
    // TODO: Then return it.

    float radian = rotation_angle * MY_PI / 180;
    Eigen::Vector3f n = {0.0, 0.0, 1.0};
    Eigen::Matrix4f model;
    Eigen::Matrix3f tr = Eigen::Matrix3f::Identity();

    //* 普通作业
    // model << cos(radian), -sin(radian), 0.0, 0.0,
    //     sin(radian), cos(radian), 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 1.0;

    //* 提高: 绕任意过原点的轴旋转
    Eigen::Matrix3f N;
    N << 0.0, -n.z(), n.y(),
        n.z(), 0.0, -n.x(),
        -n.y(), n.x(), 0.0;

    tr = cos(radian) * tr + (1.0 - cos(radian)) * n * n.transpose() + sin(radian) * N;

    model << tr(0, 0), tr(0, 1), tr(0, 2), 0,
        tr(1, 0), tr(1, 1), tr(1, 2), 0,
        tr(2, 0), tr(2, 1), tr(2, 2), 0,
        0, 0, 0, 1;

    return model;
}

//  r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // TODO: Implement this function
    // TODO: Create the projection matrix for the given parameters.
    // TODO: Then return it.
    //* right left top buttom near far
    float r, l, t, b = 0.0;
    t = tan(eye_fov / 2.0) * abs(zNear);
    r = t * aspect_ratio;
    l = -r;
    b = -t;

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate << 1.0, 0.0, 0.0, -(r + l) / 2.0,
        0.0, 1.0, 0.0, -(t + b) / 2.0,
        0.0, 0.0, 1.0, -(zNear + zFar) / 2.0,
        0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale << 2.0 / (r - l), 0.0, 0.0, 0.0,
        0.0, 2.0 / (t - b), 0.0, 0.0,
        0.0, 0.0, 2.0 / (zNear - zFar), 0.0,
        0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f orthographic = Eigen::Matrix4f::Identity();
    orthographic = scale * translate;

    Eigen::Matrix4f perspective = Eigen::Matrix4f::Identity();
    perspective << zNear, 0.0, 0.0, 0.0,
        0.0, zNear, 0.0, 0.0,
        0.0, 0.0, zNear + zFar, -zNear * zFar,
        0.0, 0.0, -1.0, 0.0;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection = orthographic * perspective;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; //* world space

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << key << '\n';

        if (key == 87)
        {
            angle += 10;
        }
        else if (key == 83)
        {
            angle -= 10;
        }
    }
    return 0;
}
