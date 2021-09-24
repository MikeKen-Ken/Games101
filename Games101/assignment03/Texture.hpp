//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        if (v > 1)
            v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        if (v > 1)
            v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float minU = std::floor(u_img);
        float maxU = std::ceil(u_img);
        float minV = std::floor(v_img);
        float maxV = std::ceil(v_img);

        auto color1 = image_data.at<cv::Vec3b>(minV, minU);
        auto color2 = image_data.at<cv::Vec3b>(maxV, minU);
        auto color3 = image_data.at<cv::Vec3b>(minV, maxU);
        auto color4 = image_data.at<cv::Vec3b>(maxV, maxU);

        float learpC1 = u_img - minU;
        float learpC2 = v_img - minV;
        auto lerp1 = color1 * (1 - learpC1) + color3 * learpC1;
        auto lerp2 = color2 * (1 - learpC1) + color4 * learpC1;
        auto lerp3 = lerp1 * (1 - learpC2) + lerp2 * learpC2;

        return Eigen::Vector3f(lerp3[0], lerp3[1], lerp3[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
