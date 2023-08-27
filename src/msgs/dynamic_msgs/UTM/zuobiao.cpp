#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

void positionGPStoMeters(const double &longitude, const double &latitude, double &pose_x, double &pose_y) {
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
