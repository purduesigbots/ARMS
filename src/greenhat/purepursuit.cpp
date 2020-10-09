#include "greenhat/purepursuit.h"
#include "api.h"
#include "greenhat/drive.h"
using namespace pros;

namespace purepursuit {
double m,a,b,c;
double x1,x2,x3,y1,y2,y3;
double x_int_1, x_int_2, y_int_1, y_int_2;
double last_point[3];
double second_last_point[3];
std::vector<std::array<double, 3>> intersection_points;
std::vector<double> target_point;

std::vector<double> findIntersectionPoint(double path[][2], double radius) {
  intersection_points.clear();
  x1 = greenhat::global_x;
  y1 = greenhat::global_y;

  while (intersection_points.size() == 0) {
    for (int i = 1; i < (sizeof(path) / sizeof(path[0])); i++) {
      x2 = path[i-1][0];
      y2 = path[i-1][1];
      x3 = path[i][0];
      y3 = path[i][1];

      m = (y3 - y2) / (x3 - x2);
      if (isfinite(m)) {
        a = x1 + m*m*x2 - m*y2 + m*y1;
        b = sqrt(-m*m*x1*x1 + 2*m*m*x1*x2 + 2*m*x1*y1 - 2*m*x1*y2 + m*m*radius*radius + 2*m*x2*y2 + radius*radius + 2*y2*y1 - m*m*x2*x2 - 2*m*x2*y1 - y2*y2 - y1*y1);
        c = 1 + m*m;

        x_int_1 = (a - b) / c;
        x_int_2 = (a + b) / c;

        if (isfinite(x_int_1)) {
          if ((x2 < x3 && x_int_1 < x3 && x_int_1 > x2) || (x2 > x3 && x_int_1 > x3 && x_int_1 < x2))
            y_int_1 = m * (x_int_1 - x2) + y2;
            intersection_points.push_back({(double)i, x_int_1, y_int_1});
        }
        if (isfinite(x_int_2)) {
          if ((x2 < x3 && x_int_2 < x3 && x_int_2 > x2) || (x2 > x3 && x_int_2 > x3 && x_int_2 < x2))
            y_int_2 = m * (x_int_2 - x2) + y2;
            intersection_points.push_back({(double)i, x_int_2, y_int_2});
        }
      }
      else {
        a = sqrt(radius*radius - x2*x2 + 2*x2*x1 - x1*x1);
        y_int_1 = x1 - a;
        y_int_2 = x1 + a;
        x_int_1 = x2;

        if (isfinite(y_int_1)) {
          if ((x2 < x3 && x_int_1 < x3 && x_int_1 > x2) || (x2 > x3 && x_int_1 > x3 && x_int_1 < x2))
            intersection_points.push_back({(double)i, x_int_1, y_int_1});
        }
        if (isfinite(y_int_2)) {
          if ((x2 < x3 && x_int_2 < x3 && x_int_2 > x2) || (x2 > x3 && x_int_2 > x3 && x_int_2 < x2))
            intersection_points.push_back({(double)i, x_int_1, y_int_2});
        }
      }
    }
    radius ++;
  }

  for (int i = 0; i < 4; i++) {
    last_point[i] = intersection_points.at(intersection_points.size() - 1)[i];
    second_last_point[i] = intersection_points.at(intersection_points.size() - 2)[i];
  }

  if (second_last_point[0] == last_point[0]) {
    if (x3 > x2) {
      if (last_point[1] > second_last_point[1]) {
        target_point.push_back(last_point[1]);
        target_point.push_back(last_point[2]);
      }
      else {
        target_point.push_back(second_last_point[1]);
        target_point.push_back(second_last_point[2]);
      }
    }
    else if (x3 < x2) {
      if (last_point[1] < second_last_point[1]) {
        target_point.push_back(last_point[1]);
        target_point.push_back(last_point[2]);
      }
      else {
        target_point.push_back(second_last_point[1]);
        target_point.push_back(second_last_point[2]);
      }
    }
    else {
      if (y3 > y2) {
        if (last_point[2] > second_last_point[2]) {
          target_point.push_back(last_point[1]);
          target_point.push_back(last_point[2]);
        }
        else {
          target_point.push_back(second_last_point[1]);
          target_point.push_back(second_last_point[2]);
        }
      }
      else {
        if (last_point[2] < second_last_point[2]) {
          target_point.push_back(last_point[1]);
          target_point.push_back(last_point[2]);
        }
        else {
          target_point.push_back(second_last_point[1]);
          target_point.push_back(second_last_point[2]);
        }
      }
    }
  }
  else {
    target_point.push_back(last_point[1]);
    target_point.push_back(last_point[2]);
  }

  return target_point;
}
} // namespace purepursuit
