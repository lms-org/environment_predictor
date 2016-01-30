#include <string>

#include "environment_predictor.h"
#include "lms/imaging/converter.h"
#include <lms/imaging/image_factory.h>
#include "lms/imaging/warp.h"
#include "street_environment/road.h"
#include "street_environment/crossing.h"
#include "cmath"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"
#include "local_course/local_course.h"

bool EnvironmentPredictor::initialize() {
    envInput = readChannel<street_environment::EnvironmentObjects>("ENVIRONMENT_INPUT");

    roadOutput = writeChannel<street_environment::RoadLane>("ROAD_OUTPUT");
    debugPoints = writeChannel<lms::math::polyLine2f>("DEBUG_POINTS");
    car = readChannel<sensor_utils::Car>("CAR");
    return true;
}

bool EnvironmentPredictor::deinitialize() {
    return true;
}

bool EnvironmentPredictor::cycle() {


    lms::ServiceHandle<local_course::LocalCourse> localCourse = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE");
    if(getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE")->rcStateChanged()){
        localCourse->resetData();
        logger.error("reset kalman");
        debugPoints->points().clear();
    }else{
        //Add new points
        for(const street_environment::EnvironmentObjectPtr obj :envInput->objects){
            if(obj->name().find("LANE") == std::string::npos){
                //no valid lane, some other env object!
                continue;
            }
            const street_environment::RoadLanePtr rl = std::static_pointer_cast<street_environment::RoadLane>(obj);
            if(rl->points().size() == 0)
                continue;
            if(rl->type() == street_environment::RoadLaneType::LEFT){
                logger.debug("cycle") << "found left lane: " << rl->points().size();
                localCourse->addPoints(rl->moveOrthogonal(0.4).points());
            }else if(rl->type() == street_environment::RoadLaneType::RIGHT){
                logger.debug("cycle") << "found right lane: " << rl->points().size();
                localCourse->addPoints(rl->moveOrthogonal(-0.4).points());
            }else if(rl->type() == street_environment::RoadLaneType::MIDDLE){
                logger.debug("cycle") << "found middle lane: " << rl->points().size();
                localCourse->addPoints(rl->points());
            }
        }

        debugPoints->points() = localCourse->getPointsToAdd();

        float r_fakt_min = config().get<float>("r_fakt_min", 15);
        float r_fakt_max = config().get<float>("r_fakt_max", 150);
        float velocity_max = config().get<float>("r_fakt_maxVelocity", 4.0);;
        float r_fakt = std::max(static_cast<double>(r_fakt_min), r_fakt_max - fabs(car->velocity())*(r_fakt_max - r_fakt_min)/velocity_max);
        //TODO
        if(config().get<bool>("translateEnvironment",false)){
            logger.info("translation")<<car->deltaPhi();
            //localCourse->update(car->localDeltaPosition().x,car->localDeltaPosition().y,car->deltaPhi()); //TODO x and y translation produce bad results
            localCourse->update(0.0,0.0,car->deltaPhi(), r_fakt);
        }else{
            localCourse->update(0,0,0, r_fakt);
        }
    }
    *roadOutput = localCourse->getCourse();
    roadOutput->type(street_environment::RoadLaneType::MIDDLE);
    return true;
}

