#ifndef ENVIRONMENT_PREDICTOR_H
#define ENVIRONMENT_PREDICTOR_H

#include "lms/module.h"
#include "lms/imaging/image.h"
#include "lms/imaging/format.h"
#include "street_environment/road.h"
#include "lms/type/module_config.h"
#include "sensor_utils/car.h"
#include <fstream>

#include "kalman_filter_lr_emxAPI.h"


class EnvironmentPredictor : public lms::Module {

public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    void resetData();
    lms::ReadDataChannel<street_environment::EnvironmentObjects> envInput;
    lms::WriteDataChannel<street_environment::RoadLane> roadOutput;
    lms::ReadDataChannel<sensor_utils::Car> car;
    lms::WriteDataChannel<lms::math::polyLine2f> debugPoints;
};

#endif /* IMAGE_CONVERTER_H */
