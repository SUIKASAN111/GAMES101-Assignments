//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0)u = 0;
        if (u > 1)u = 1;
        if (v < 0)v = 0;
        if (v > 1)v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinearInterpolation(float u, float v)
    {
        if (u < 0)u = 0;
        if (u > 1)u = 1;
        if (v < 0)v = 0;
        if (v > 1)v = 1;

        float w00 = std::floor(u * width), h00 = std::floor((1-v) * height);
        float w10 = w00 + 1.0f, h10 = h00;
        float w01 = w00, h01 = h00 + 1.0f;
        float w11 = w00 + 1.0f, h11 = h00 + 1.0f;

        auto color00 = image_data.at<cv::Vec3b>(w00, h00);
        auto color10 = image_data.at<cv::Vec3b>(w10, h10);
        auto color01 = image_data.at<cv::Vec3b>(w01, h01);
        auto color11 = image_data.at<cv::Vec3b>(w11, h11);
        auto color0 = color00 + (color10 - color00) * (u * width - w00);
        auto color1 = color01 + (color11 - color01) * (u * width - w01);//w01==w00
        auto color = color0 + (color1 - color0) * ((1 - v) * height - h00);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
