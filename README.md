# environment_predictor
Module that calls LOCAL_COURSE_SERVICE with road-points merged on the middle-lane

## DataChannels
- DEBUG_POINTS_RAW unfiltered points used by the service
- DEBUG_POINTS by the service filtered points
- ROAD_OUTPUT created road
- ENVIRONMENT_INPUT lanes of the road that will be merged and given to the service

##Dependencies
 * math
 * street_environment
 * sensor_utils
