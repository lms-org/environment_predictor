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
    debugPointsRaw = writeChannel<lms::math::polyLine2f>("DEBUG_POINTS_RAW");
    car = readChannel<street_environment::CarCommand>("CAR");
    return true;
}

bool EnvironmentPredictor::deinitialize() {
    return true;
}

bool EnvironmentPredictor::cycle() {

    lms::ServiceHandle<local_course::LocalCourse> localCourse = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE");

    //check if we need to reset the environment
    lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> phoenixService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
    if(phoenixService->rcStateChanged() || phoenixService->driveModeChanged()){
        //phoenixService->logRcStates();
        localCourse->resetData();
        logger.error("reset kalman");
        debugPoints->points().clear();
    }else{
        //update the service with new points
        for(const street_environment::EnvironmentObjectPtr obj :envInput->objects){
            if(obj->name().find("LANE") == std::string::npos){
                //no valid lane, some other env object!
                continue;
            }
            //add the lines and translate them if necessary
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
        //get used points by the localCourse for debugging
        debugPointsRaw->points()=  localCourse->getPointsToAdd();
        //update the environment
        if(config().get<bool>("translateEnvironment",false)){
            logger.info("translation")<<car->deltaPhi();
            //localCourse->update(car->localDeltaPosition().x,car->localDeltaPosition().y,car->deltaPhi()); //TODO x and y translation produce bad results
            double maxYawRate = 0.03;
            double deltaPhi = car->deltaPhi();
            //TODO is there still a fail?
            logger.error("deltaPhi") << deltaPhi;
            if (deltaPhi < -maxYawRate)
                deltaPhi = -maxYawRate;
            else if (deltaPhi > maxYawRate)
                deltaPhi = maxYawRate;
            logger.time("localCourse");
            localCourse->update(0,0,deltaPhi);
            logger.timeEnd("localCourse");
        }else{
            logger.time("localCourse");
            localCourse->update(0,0,0);
            logger.timeEnd("localCourse");
        }
    }
    //create data-output
    street_environment::RoadLane out = localCourse->getCourse();
    bool valid = true;
    for(const lms::math::vertex2f &v:out.points()){
        if(std::isnan(v.x) || std::isnan(v.y)){
            logger.error("cycle")<<"localcourse returned invalid road, element is nan!";
            valid = false;
        }
    }
    if(valid){
        *roadOutput = out;
    }else{
        logger.error("cycle")<<"using old roadOutput as new one is invalid!";
        //TODO error handling!
    }
    roadOutput->type(street_environment::RoadLaneType::MIDDLE);


    debugPoints->points() = localCourse->getPointsAdded();

    return true;
}

