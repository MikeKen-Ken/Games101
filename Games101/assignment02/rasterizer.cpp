//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

double sign_func(double x)
{
    if (x > 0)
        return +1.0;
    else if (x == 0)
        return 0.0;
    else
        return -1.0;
}

static bool insideTriangle(float x, float y, const Vector3f *_v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f point = {x, y, 0.0};
    float sign1 = (point - _v[0]).cross(_v[1] - _v[0]).z();
    float sign2 = (point - _v[1]).cross(_v[2] - _v[1]).z();
    float sign3 = (point - _v[2]).cross(_v[0] - _v[2]).z();
    if (sign_func(sign1) == sign_func(sign2) && sign_func(sign2) == sign_func(sign3))
    {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        //Homogeneous division
        for (auto &vec : v)
        {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto &vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();

    //* 取aabb包围盒
    int min_x = MIN(v[0].x(), MIN(v[1].x(), v[2].x()));
    int max_x = MAX(v[0].x(), MAX(v[1].x(), v[2].x()));
    int min_y = MIN(v[0].y(), MIN(v[1].y(), v[2].y()));
    int max_y = MAX(v[0].y(), MAX(v[1].y(), v[2].y()));

    float alpha;
    float beta;
    float gamma;
    float w_reciprocal;
    float z_interpolated;
    int index;
    Vector3f color = t.getColor();
    Vector3f point;
    Vector3f colorWeight;
    float weight;

    bool SSAA = false;

    // TODO :Find out the bounding box of current triangle.
    // TODO :iterate through the pixel and find if the current pixel is inside the triangle
    //! 因为最大的像素点被int取整了一次，此处使用<=保证可以取到最远的像素点
    for (int x = min_x; x <= max_x; x++)
    {
        for (int y = min_y; y <= max_y; y++)
        {
            //* 提高 这里需要对子采样点分别维护子采样点对应的深度列表和颜色列表
            if (SSAA)
            {
                colorWeight = {0, 0, 0};
                weight = 0;
                for (int xi = 1, xTimes = 0; xi < 4; xi += 2, xTimes++)
                {
                    for (int yi = 1, yTimes = 0; yi < 4; yi += 2, yTimes++)
                    {
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(x + xi * 0.25, y + yi * 0.25, t.v);
                        w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        index = get_specific_index(x * 2 + xTimes, y * 2 + yTimes);
                        //*　共四种情况
                        //* 1.三角形内 距离视点近     采用近处也就是本三角形的t.color
                        //* 2.三角形内 距离视点远     采用远处缓冲区的color
                        //* 3.三角形外 距离视点近     采用远处缓冲区的color
                        //* 4.三角形外 距离视点远     采用远处缓冲区的color
                        if (z_interpolated < subdepth_buf[index] && insideTriangle(x + xi * 0.25, y + yi * 0.25, t.v))
                        {
                            subdepth_buf[index] = z_interpolated;
                            color_buf[index] = color;
                            colorWeight += color;
                        }
                        //* 采用远处缓冲区的color
                        else
                        {
                            colorWeight += color_buf[index];
                        }
                        weight++;
                    }
                }
                point << (float)x, (float)y, 1.0;
                set_pixel(point, colorWeight / weight);
            }
            else
            {
                // TODO :If so, use the following code to get the interpolated z value.
                //* 这里取像素中点判断是否在三角形内
                if (insideTriangle(x + 0.5, y + 0.5, t.v))
                {
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    z_interpolated = -z_interpolated;
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    //* deptp_buf 初始值为+inf，先转换z_value为正值，之后再做比较
                    if (z_interpolated < depth_buf[get_index(x, y)])
                    {
                        point << (float)x, (float)y, 1.0;
                        depth_buf[get_index(x, y)] = z_interpolated;
                        set_pixel(point, color);
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{255, 0, 0});
        std::fill(color_buf.begin(), color_buf.end(), Eigen::Vector3f{255, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(subdepth_buf.begin(), subdepth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //* 子采样点深度列表
    subdepth_buf.resize(w * h * 4);
    //* 子采样点颜色列表
    color_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    // old return (height - 1 - y) * width + x;
    return x + std::max(0, (y - 1)) * width;
}

int rst::rasterizer::get_specific_index(int x, int y)
{
    // old  return (height * 2 - 1 - y) * width * 2 + x;
    return x + std::max(0, (y - 1)) * width;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on