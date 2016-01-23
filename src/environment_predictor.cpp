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
        return true;
    }else{
        //Add new points
        for(const std::shared_ptr<const street_environment::EnvironmentObject> obj :envInput->objects){
            if(obj->name().find("LANE") == std::string::npos){
                //no valid lane, some other env object!
                continue;
            }
            const street_environment::RoadLane &rl = obj->getAsReference<const street_environment::RoadLane>();
            if(rl.points().size() == 0)
                continue;
            if(rl.type() == street_environment::RoadLaneType::LEFT){
                logger.debug("cycle") << "found left lane: " << rl.points().size();
                localCourse->addPoints(rl.moveOrthogonal(0.4).points());
            }else if(rl.type() == street_environment::RoadLaneType::RIGHT){
                logger.debug("cycle") << "found right lane: " << rl.points().size();
                localCourse->addPoints(rl.moveOrthogonal(-0.4).points());
            }else if(rl.type() == street_environment::RoadLaneType::MIDDLE){
                logger.debug("cycle") << "found middle lane: " << rl.points().size();
                localCourse->addPoints(rl.points());
            }
        }

        debugPoints->points() = localCourse->getPointsToAdd();
        //TODO
        //logger.info("translation")<<<<car->localDeltaPosition()<<" "<<car->deltaPhi();
        if(config().get<bool>("translateEnvironment",false)){
        localCourse->update(car->localDeltaPosition().x,car->localDeltaPosition().y,car->deltaPhi());
        }else{
            localCourse->update(0,0,0);
        }
    }
    *roadOutput = localCourse->getCourse();
    roadOutput->type(street_environment::RoadLaneType::MIDDLE);
    return true;
}

