
#ifndef FUSION_KF_H_
#define FUSION_KF_H_

#include <fstream>
#include <string>
#include <vector>
#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking
{
  public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage& measurement_pack);
    KalmanFilter kf_;

  private:
    bool is_initialized_;
    long previous_timestamp_;

    // acceleration noise components
    float noise_ax;
    float noise_ay;
};

#endif /* FUSION_KF_H_ */
