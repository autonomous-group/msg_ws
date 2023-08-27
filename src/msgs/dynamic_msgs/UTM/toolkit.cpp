//
// Created by yao on 20-3-23.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "toolkit.h"

int ToolKit::getCurveLength(const std::vector<double> &local_x,
                            const std::vector<double> &local_y,
                            std::vector<double> *ptr_local_s,
                            std::vector<double> *ptr_arc_length,
                            const double &s_limit) {
    assert(local_x.size() > 2 && local_x.size() == local_y.size());
    ptr_local_s->clear();
    ptr_arc_length->clear();
    double s = 0.0, delta_s = 0.0;

    for (unsigned int i = 0; i < local_x.size(); ++i) {
        if (i != 0) {
            delta_s = sqrt(pow(local_x[i] - local_x[i - 1], 2) + pow(local_y[i] - local_y[i - 1], 2));
            assert(delta_s > 0.01);
        }
        s += delta_s;
        ptr_local_s->emplace_back(s);
        ptr_arc_length->emplace_back(delta_s);
        if (ptr_local_s->back() > s_limit)
            break;
    }
    return ptr_local_s->size();
}

void ToolKit::getHeading(const std::vector<double> &local_x,
                         const std::vector<double> &local_y,
                         std::vector<double> *pt_heading_out) {
    assert(local_x.size() > 2 && local_x.size() == local_y.size());
    unsigned long size_n = local_x.size();

    std::vector<double> line_heading(size_n - 1, 0.);
    for (int i = 0; i < size_n - 1; ++i) {
        line_heading[i] = atan2(local_y[i + 1] - local_y[i], local_x[i + 1] - local_x[i]);
    }

    std::vector<double> delta_heading(size_n, 0.);
    for (int i = 1; i < size_n - 1; ++i) {
        delta_heading[i] = (line_heading[i] - line_heading[i - 1]) / 2.;
    }
    delta_heading[0] = delta_heading[1];
    delta_heading[size_n - 1] = delta_heading[size_n - 2];

    pt_heading_out->clear();
    pt_heading_out->resize(size_n);
    for (int i = 0; i < size_n - 1; ++i)
        pt_heading_out->at(i) = line_heading[i] - delta_heading[i];
    pt_heading_out->at(size_n - 1) = line_heading[size_n - 2] + delta_heading[size_n - 1];
}

void ToolKit::getCurvature(const std::vector<double> &local_x,
                           const std::vector<double> &local_y,
                           const std::vector<double> &arc_length,
                           const std::vector<double> &heading,
                           std::vector<double> *curvature) {
    assert(local_x.size() > 2 && local_x.size() == local_y.size() && local_x.size() == heading.size()
               && local_x.size() == arc_length.size() + 1);
    unsigned long size_n = local_x.size();

    std::vector<double> line_heading(size_n - 1, 0.);
    for (int i = 0; i < size_n - 1; ++i) {
        line_heading[i] = atan2(local_y[i + 1] - local_y[i], local_x[i + 1] - local_x[i]);
    }
    std::vector<double> delta_heading(size_n, 0.);
    for (int i = 0; i < size_n - 1; ++i) {
        delta_heading[i] = line_heading[i] - heading[i];
    }
    delta_heading[size_n - 1] = delta_heading[size_n - 2];

    curvature->clear();
    curvature->resize(size_n);
    for (int i = 0; i < size_n - 1; ++i)
        curvature->at(i) = 2 * sin(delta_heading[i]) / arc_length[i];
    curvature->at(size_n - 1) = curvature->at(size_n - 2);
}

void ToolKit::getHeadingCurvature(const std::vector<double> &local_x,
                                  const std::vector<double> &local_y,
                                  const std::vector<double> &arc_length,
                                  std::vector<double> *heading,
                                  std::vector<double> *curvature) {
    assert(local_x.size() > 2 && local_x.size() == local_y.size() && local_x.size() == arc_length.size() + 1);
    unsigned long size_n = local_x.size();

    std::vector<double> line_heading(size_n - 1, 0.);
    for (int i = 0; i < size_n - 1; ++i) {
        line_heading[i] = atan2(local_y[i + 1] - local_y[i], local_x[i + 1] - local_x[i]);
    }
    std::vector<double> delta_heading(size_n, 0.);
    for (int i = 1; i < size_n - 1; ++i) {
        delta_heading[i] = (line_heading[i] - line_heading[i - 1]) / 2.;
    }
    delta_heading[0] = delta_heading[1];
    delta_heading[size_n - 1] = delta_heading[size_n - 2];

    heading->clear();
    curvature->clear();
    heading->resize(size_n);
    curvature->resize(size_n);
    for (int i = 0; i < size_n - 1; ++i) {
        heading->at(i) = line_heading[i] - delta_heading[i];
        curvature->at(i) = 2 * sin(delta_heading[i]) / arc_length[i];
    }
    heading->at(size_n - 1) = line_heading[size_n - 2] + delta_heading[size_n - 1];
    curvature->at(size_n - 1) = curvature->at(size_n - 2);
}

void ToolKit::getCurvature(const std::vector<double> &local_x,
                           const std::vector<double> &local_y,
                           std::vector<double> *pt_curvature_out) {
    assert(local_x.size() == local_y.size());
    unsigned long size_n = local_x.size();
    std::vector<double> curvature = std::vector<double>(size_n);
    for (int i = 1; i < size_n - 1; ++i) {
        double x1 = local_x.at(i - 1);
        double x2 = local_x.at(i);
        double x3 = local_x.at(i + 1);
        double y1 = local_y.at(i - 1);
        double y2 = local_y.at(i);
        double y3 = local_y.at(i + 1);
        curvature.at(i) = getPointCurvature(x1, y1, x2, y2, x3, y3);
    }
    //ldr20200705：路网第一点曲率与第二点曲率相同，路网最后一点与倒数第二点曲率相同。
    curvature.at(0) = curvature.at(1);
    curvature.at(size_n - 1) = curvature.at(size_n - 2);
    pt_curvature_out->clear();
    for (int j = 0; j < size_n; ++j) {
        if (j == 0 || j == size_n - 1)
            pt_curvature_out->emplace_back(curvature[j]);
        else
            pt_curvature_out->emplace_back((curvature[j - 1] + curvature[j] + curvature[j + 1]) / 3);
    }
}

double ToolKit::getPointCurvature(const double &x1, const double &y1,
                                  const double &x2, const double &y2,
                                  const double &x3, const double &y3) {
    double_t a, b, c;
    double_t delta_x, delta_y;
    double_t s;
    double_t A;
    double_t curv;
    double_t rotate_direction;

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    a = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    delta_x = x3 - x2;
    delta_y = y3 - y2;
    b = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    delta_x = x1 - x3;
    delta_y = y1 - y3;
    c = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    s = (a + b + c) / 2.0;
    A = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
    curv = 4 * A / (a * b * c);

    /* determine the sign, using cross product(叉乘)
     * 2维空间中的叉乘是： A x B = |A||B|Sin(\theta)
     * V1(x1, y1) X V2(x2, y2) = x1y2 – y1x2
     */
    rotate_direction = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    //TODO: 叉积极小时，比如0.001，可以让曲率为零
    if (rotate_direction < 0) {
        curv = -curv;
    }
    return curv;
}

void ToolKit::positionGPStoMeters(const double &longitude, const double &latitude, double &pose_x, double &pose_y) {
    static int e0 = 0;
    static int n0 = 0;
    double WGS84_ECCENTRICITY = (double) 0.0818192;                          // e=0.0818192
    double WGS84_EQUATORIAL_RADIUS = (double) 6378.137;                      // a=6378.137
    double k0 = (double) 0.9996;

    int Zone = (int) (longitude / 6) + 1;
    int lonBase = Zone * 6 - 3;

    double vPhi = (double) (1 / sqrt(1 - pow(WGS84_ECCENTRICITY * sin(latitude * M_PI / 180.0), 2)));
    double A = (double) ((longitude - lonBase) * M_PI / 180.0 * cos(latitude * M_PI / 180.0));
    double sPhi = (double) ((1 - pow(WGS84_ECCENTRICITY, 2) / 4.0 - 3 * pow(WGS84_ECCENTRICITY, 4) / 64.0
        - 5 * pow(WGS84_ECCENTRICITY, 6) / 256.0) * latitude * M_PI / 180.0
        - (3 * pow(WGS84_ECCENTRICITY, 2) / 8.0 + 3 * pow(WGS84_ECCENTRICITY, 4) / 32.0
            + 45 * pow(WGS84_ECCENTRICITY, 6) / 1024.0) * sin(2 * latitude * M_PI / 180.0)
        + (15 * pow(WGS84_ECCENTRICITY, 4) / 256.0 + 45 * pow(WGS84_ECCENTRICITY, 6) / 256.0)
            * sin(4 * latitude * M_PI / 180.0)
        - (35 * pow(WGS84_ECCENTRICITY, 6) / 3072.0) * sin(6 * latitude * M_PI / 180.0));
    double T = (double) (pow(tan(latitude * M_PI / 180.0), 2));
    double C = (double) ((pow(WGS84_ECCENTRICITY, 2) / (1 - pow(WGS84_ECCENTRICITY, 2)))
        * pow(cos(latitude * M_PI / 180.0), 2));

    pose_x = (double) ((k0 * WGS84_EQUATORIAL_RADIUS * vPhi * (A + (1 - T + C) * pow(A, 3) / 6.0
        + (5 - 18 * T + pow(T, 2)) * pow(A, 5) / 120.0)) * 1000);
    pose_y =
        (double) ((k0 * WGS84_EQUATORIAL_RADIUS * (sPhi + vPhi * tan(latitude * M_PI / 180.0) * (pow(A, 2) / 2
            + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24.0 + (61 - 58 * T + T * T) * pow(A, 6) / 720.0))) * 1000);
    if (0 == e0 && 0 == n0) {
        e0 = int(pose_x);
        n0 = int(pose_y);
    }
    pose_x -= e0;
    pose_y -= n0;
}

void ToolKit::fillPoints(std::vector<double> &local_path_x, std::vector<double> &local_path_y, const double &length) {
    if (local_path_x.size() < 2)
        return;
    double interval = sqrt(pow(local_path_x[local_path_x.size() - 1] - local_path_x[local_path_x.size() - 2], 2)
                               + pow(local_path_y[local_path_y.size() - 1] - local_path_y[local_path_y.size() - 2], 2));
    interval = std::max(interval, 0.5);
    int max_points = length / interval + 1;
    for (int i = 0; i < max_points; ++i) {
        local_path_x.push_back(2 * local_path_x[local_path_x.size() - 1] - local_path_x[local_path_x.size() - 2]);
        local_path_y.push_back(2 * local_path_y[local_path_y.size() - 1] - local_path_y[local_path_y.size() - 2]);
    }
}
