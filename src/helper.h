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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

// Calculate the euclidean distance between two points
double distance(double x1, double y1, double x2, double y2);

// Get the closest waypoint of the track (could be behind or in a bad direction)
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

// Get the next waypoint of the track
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* HELPER_H */
