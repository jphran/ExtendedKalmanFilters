//
// Created by jfrancis on 3/12/21.
//

#ifndef EXTENDEDKALMANFILTERS_MEASUREMENT_PACKAGE_H
#define EXTENDEDKALMANFILTERS_MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
public:

    enum SensorType {
        LASER, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    int64_t timestamp_;

};

#endif //EXTENDEDKALMANFILTERS_MEASUREMENT_PACKAGE_H
