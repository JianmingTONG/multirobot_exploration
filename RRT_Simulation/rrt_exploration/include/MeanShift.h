#pragma once 

#include <vector>

class MeanShift {
public:
    typedef std::vector<double> Point;

    MeanShift() { set_kernel(NULL); }
    MeanShift(double (*_kernel_func)(double,double)) { set_kernel(kernel_func); }
    std::vector<Point> meanshift(const std::vector<Point> & points,
                                                double kernel_bandwidth,
                                                double EPSILON = 0.00001);
    void meanshift_center(const std::vector<Point> &points,
                                            std::vector<Point> &out,
                                             double kernel_bandwidth,
                                             double EPSILON = 0.00001);

private:
    double (*kernel_func)(double,double);
    void set_kernel(double (*_kernel_func)(double,double));
    void shift_point(const Point&, const std::vector<Point> &, double, Point&);
};
