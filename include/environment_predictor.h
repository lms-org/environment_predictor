#ifndef ENVIRONMENT_PREDICTOR_H
#define ENVIRONMENT_PREDICTOR_H

#include "lms/module.h"
#include "lms/imaging/image.h"
#include "lms/imaging/format.h"
#include "street_environment/road.h"
#include "lms/config.h"
#include "street_environment/car.h"
#include <fstream>


class EnvironmentPredictor : public lms::Module {

public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    void resetData();
    lms::ReadDataChannel<street_environment::EnvironmentObjects> envInput;
    lms::WriteDataChannel<street_environment::RoadLane> roadOutput;
    lms::ReadDataChannel<street_environment::Car> car;
    lms::WriteDataChannel<lms::math::polyLine2f> debugPoints;
    lms::WriteDataChannel<lms::math::polyLine2f> debugPointsRaw;
};

#endif /* IMAGE_CONVERTER_H */
