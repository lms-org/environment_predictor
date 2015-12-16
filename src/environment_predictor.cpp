#include <string>

#include "environment_predictor.h"
#include "lms/imaging/converter.h"
#include <lms/imaging/image_factory.h>
#include "lms/imaging/warp.h"
#include "street_environment/road.h"
#include "cmath"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

#include "kalman_filter_lr.h"
bool EnvironmentPredictor::initialize() {
    envInput = readChannel<street_environment::EnvironmentObjects>("ENVIRONMENT_INPUT");

    roadOutput = writeChannel<street_environment::RoadLane>("ROAD_OUTPUT");
    car = readChannel<sensor_utils::Car>("CAR");

    partCount = config().get<int>("elementCount",10);
    partLength = config().get<float>("elementLength",0.2);
    zustandsVector = emxCreate_real_T(partCount,1);

    stateTransitionMatrix = emxCreate_real_T(partCount,partCount);
    kovarianzMatrixDesZustandes = emxCreate_real_T(partCount,partCount);
    kovarianzMatrixDesZustandUebergangs = emxCreate_real_T(partCount,partCount);
    cycleCounter = 0;
    if(config().get<bool>("logState", false)){
        logFile.open(config().get<std::string>("logPrefix") + "_" + lms::currentTimeString() + "_kalman.csv", std::ofstream::out);
    }
    
    resetData();
    configsChanged();


    return true;
}

void EnvironmentPredictor::resetData(){
    logger.info("resetData");
    clearMatrix(zustandsVector);
    zustandsVector->data[0] = config().get<float>("distanceToMiddle",0.2);
    asEinheitsMatrix(stateTransitionMatrix, 1);
    asEinheitsMatrix(kovarianzMatrixDesZustandes, 1);
    asEinheitsMatrix(kovarianzMatrixDesZustandUebergangs, config().get<float>("kov",15));
    //clearMatrix(kovarianzMatrixDesZustandUebergangs);

    for(int x = 0; x < partCount; x++){
        for(int y = 0; y < partCount; y++){
            if (x != 0 && y != 0) kovarianzMatrixDesZustandUebergangs->data[y*partCount+x]=config().get<float>("kov",15)*(1-pow(config().get<float>("kovAbnahme",0.2),1/fabs(x-y)));
        }
    }
}

bool EnvironmentPredictor::deinitialize() {
    return true;
}

bool EnvironmentPredictor::cycle() {

    //I don't like it hmmm
    if(getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE")->rcStateChanged()){
        resetData();
    }

    r_fakt=config().get<double>("r_fakt",20);

    //länge der später zu berechnenden Abschnitten
    //convert data to lines
    emxArray_real_T *rx = nullptr;
    emxArray_real_T *ry = nullptr;
    emxArray_real_T *lx = nullptr;
    emxArray_real_T *ly = nullptr;
    emxArray_real_T *mx = nullptr;
    emxArray_real_T *my = nullptr;
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
            convertToKalmanArray(rl,&lx,&ly);
        }else if(rl.type() == street_environment::RoadLaneType::RIGHT){
            logger.debug("cycle") << "found right lane: " << rl.points().size();
            convertToKalmanArray(rl,&rx,&ry);
        }else if(rl.type() == street_environment::RoadLaneType::MIDDLE){
            logger.debug("cycle") << "found middle lane: " << rl.points().size();
            convertToKalmanArray(rl,&mx,&my);
        }
    }
    if(rx == nullptr){
        logger.debug("right has zero points");
        rx = emxCreate_real_T(0,0);
        ry = emxCreate_real_T(0,0);
    }
    if(lx == nullptr){
        logger.debug("left has zero points");
        lx = emxCreate_real_T(0,0);
        ly = emxCreate_real_T(0,0);
    }
    if(mx == nullptr){
        logger.debug("middle has zero points");
        mx = emxCreate_real_T(0,0);
        my = emxCreate_real_T(0,0);
    }
    //Kalman with middle-lane
    //TODO später mit der richtigen positionsänderung arbeiten

    double deltaX = 0;
    double deltaY = 0;
    double deltaPhi =0;
    /*
    if(false || config->get<bool>("enableMove",false)){
        deltaX = car->deltaX();
        deltaY = car->deltaY();
        deltaPhi = car->deltaPhi();
    }
    */
    float prior_fact = 0; //config().get<float>("prior_fact",0);
    kalman_filter_lr(zustandsVector,deltaX,deltaY,deltaPhi,kovarianzMatrixDesZustandes,
                     kovarianzMatrixDesZustandUebergangs,
                     r_fakt,partLength,lx,ly,rx,ry,mx,my,1,prior_fact);
    createOutput();
    //destroy stuff
    emxDestroyArray_real_T(rx);
    emxDestroyArray_real_T(ry);
    emxDestroyArray_real_T(lx);
    emxDestroyArray_real_T(ly);
    emxDestroyArray_real_T(mx);
    emxDestroyArray_real_T(my);
    
    logStateVector();
    
    cycleCounter++;

    //getchar();
    
    return true;
}

void EnvironmentPredictor::logStateVector()
{
    if(!logFile)
    {
        return;
    }
    logFile << cycleCounter;
    for(int i = 0; i < zustandsVector->size[0]; i++)
    {
        logFile << "," << zustandsVector->data[i];
    }
    logFile << std::endl;
}

void EnvironmentPredictor::createOutput(){
    //create middle
    logger.debug("createOutput");
    roadOutput->type(street_environment::RoadLaneType::MIDDLE);
    convertZustandToLane(*(roadOutput.get()));
    roadOutput->name("MIDDLE_LANE");
}

void EnvironmentPredictor::convertZustandToLane(street_environment::RoadLane &output){
    //clear points
    logger.debug("convertZustandToLane ANFANG");
    output.points().clear();
    logger.debug("convertZustandToLane ANFANG")<<1;
    output.polarDarstellung.clear();
    logger.debug("convertZustandToLane CLEARED OLD VALS");

    lms::math::vertex2f p1;
    p1.x = 0;
    p1.y = zustandsVector->data[0];
    lms::math::vertex2f p2;
    p2.x = partLength*cos(zustandsVector->data[1]);
    p2.y = p1.y + partLength*sin(zustandsVector->data[1]);
    double phi = zustandsVector->data[1];
    //add points to lane
    output.points().push_back(p1);
    output.points().push_back(p2);
    output.polarPartLength = partLength;
    //add first two values
    output.polarDarstellung.push_back(zustandsVector->data[0]);
    output.polarDarstellung.push_back(zustandsVector->data[1]);

    for(int i = 2; i < partCount; i++){
        lms::math::vertex2f pi;
        double dw = 2*acos(partLength*zustandsVector->data[i]/2);
        phi = phi -dw+M_PI;
        pi.x = output.points()[i-1].x + partLength*cos(phi);
        pi.y = output.points()[i-1].y + partLength*sin(phi);
        output.points().push_back(pi);
        logger.debug("points: ")<<"krümmung: "<<zustandsVector->data[i]<< " ,dw "<<dw<<" ,x:  " <<pi.x << " ,y: "<<pi.y;
        output.polarDarstellung.push_back(zustandsVector->data[i]);
    }

}

void EnvironmentPredictor::clearMatrix(emxArray_real_T *mat){
    memset(mat->data,0,mat->size[0]*mat->size[1]*sizeof(double));

}

void EnvironmentPredictor::asEinheitsMatrix(emxArray_real_T *mat, double val){
    clearMatrix(mat);
    for(int i = 0; i < mat->size[0]; i++){
        mat->data[i*(mat->size[0]+1)] = val;
    }
}

void EnvironmentPredictor::convertToKalmanArray(const street_environment::RoadLane &lane,emxArray_real_T **x,emxArray_real_T **y){
    int dim = lane.points().size();
    emxArray_real_T *vx = emxCreate_real_T(dim,1);
    emxArray_real_T *vy = emxCreate_real_T(dim,1);
    for(uint i=0;i < lane.points().size(); i++){
        vx->data[i] = lane.points()[i].x;
        vy->data[i] = lane.points()[i].y;
    }
    *x = vx;
    *y = vy;
}

void EnvironmentPredictor::printMat(emxArray_real_T *mat){
    std::cout<<"mat: "<<std::endl;
    for(int x = 0; x < mat->size[0];x++){
        for(int y = 0; y < mat->size[1]; y++){
            std::cout << mat->data[x*mat->size[1]+y];
            std::cout <<",";
        }
        std::cout <<std::endl;
    }
}
