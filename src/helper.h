/*
 * helper.h
 *
 * Created on: October 05, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <vector>

using namespace std;

// Convert radians and degrees
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* HELPER_H */
