#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN)
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
    std::vector<cv::Point2f> points(control_points);
    std::vector<cv::Point2f> ret;
    for (auto i = points.size(); i > 1; --i)
    {
        for (std::size_t j = 0; j < i - 1; ++j)
        {
            auto point = (1 - t) * points[j] + t * points[j + 1];
            ret.push_back(point);
        }
        if (ret.size() == 1)
            break;
        else
        {
            points = ret;
            ret.clear();
        }
    }
    return cv::Point2f(ret[0]);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        cv::Point2f pointCenter;
        pointCenter.x = std::floor(point.x + 0.5f);
        pointCenter.y = std::floor(point.y + 0.5f);
        for (int i = -1; i < 1; ++i)
        {
            for (int j = -1; j < 1; ++j)
            {
                cv::Point2f pointCurr;
                pointCurr.x = pointCenter.x + i;
                pointCurr.y = pointCenter.y + j;
                if (pointCurr.x > 700 || pointCurr.x < 0 || pointCurr.y>700 || pointCurr.y < 0)
                    continue;
                float distance = sqrt(pow((pointCurr.x - point.x), 2) + pow((pointCurr.y - point.y), 2));
                float ratio = 1 - distance / 3 * sqrt(2);
                int colorG = 255 * ratio;
                if (window.at<cv::Vec3b>(pointCurr.y, pointCurr.x)[1] < colorG)
                    window.at<cv::Vec3b>(pointCurr.y, pointCurr.x)[1] = colorG;
            }
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

        if (key == 13)
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
