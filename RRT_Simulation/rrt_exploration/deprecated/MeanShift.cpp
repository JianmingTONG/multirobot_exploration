#include <iostream>
#include <math.h>
#include "MeanShift.h"
#include<iomanip>
using namespace std;

#define CLUSTER_EPSILON 0.5

double euclidean_distance(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return sqrt(total);
}

double euclidean_distance_sqr(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return (total);
}

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

void MeanShift::shift_point(const Point &point,
                            const std::vector<Point> &points,
                            double kernel_bandwidth,
                            Point &shifted_point) {
    shifted_point.resize( point.size() ) ;
    for(int dim = 0; dim<shifted_point.size(); dim++){
        shifted_point[dim] = 0;
    }
    double total_weight = 0;
    for(int i=0; i<points.size(); i++){
        const Point& temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);
        for(int j=0; j<shifted_point.size(); j++){
            shifted_point[j] += temp_point[j] * weight;
        }
        total_weight += weight;
    }

    const double total_weight_inv = 1.0/total_weight;
    for(int i=0; i<shifted_point.size(); i++){
        shifted_point[i] *= total_weight_inv;
    }
}



std::vector<MeanShift::Point> MeanShift::meanshift(const std::vector<Point> &points,
                                             double kernel_bandwidth,
                                             double EPSILON){
    const double EPSILON_SQR = EPSILON*EPSILON;
    vector<bool> stop_moving(points.size(), false);
    vector<Point> shifted_points = points;
    double max_shift_distance;
    Point point_new;
    do {
        max_shift_distance = 0;
        for(int i=0; i<points.size(); i++){
            if (!stop_moving[i]) {
                shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
                double shift_distance_sqr = euclidean_distance_sqr(point_new, shifted_points[i]);
                if(shift_distance_sqr > max_shift_distance){
                    max_shift_distance = shift_distance_sqr;
                }
                if(shift_distance_sqr <= EPSILON_SQR) {
                    stop_moving[i] = true;
                }
                shifted_points[i] = point_new;
            }
        }
        // cout<< "max_shift_distance: " <<setprecision(5)<< sqrt(max_shift_distance) << endl;
    } while (max_shift_distance > EPSILON_SQR);
    return shifted_points;
}

void MeanShift::meanshift_center(const std::vector<Point> &points,
                                            std::vector<Point> &out,
                                             double kernel_bandwidth,
                                             double EPSILON){
    const double EPSILON_SQR = EPSILON*EPSILON;
    vector<bool> stop_moving(points.size(), false);
    vector<Point> shifted_points = points;
    double max_shift_distance;
    Point point_new;
    do {
        max_shift_distance = 0;
        for(int i=0; i<points.size(); i++){
            if (!stop_moving[i]) {
                shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
                double shift_distance_sqr = euclidean_distance_sqr(point_new, shifted_points[i]);
                if(shift_distance_sqr > max_shift_distance){
                    max_shift_distance = shift_distance_sqr;
                }
                if(shift_distance_sqr <= EPSILON_SQR) {
                    stop_moving[i] = true;
                    bool cond=true;
                    for(int k =0 ; k< out.size(); k++){
                        cond = cond & (euclidean_distance_sqr(point_new, out[k]) > 0.00001);
                        if(!cond){break;}
                    }
                    if(cond){
                        out.push_back(point_new);
                        // cout << "push back new data" << endl;
                    }
                }
                shifted_points[i] = point_new;
            }
        }
        // cout<< "max_shift_distance: " <<setprecision(5)<< sqrt(max_shift_distance) << endl;
    } while (max_shift_distance > EPSILON_SQR);
   
}
