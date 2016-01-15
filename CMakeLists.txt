set ( SOURCES
	"src/interface.cpp"
        "src/environment_predictor.cpp"
)

set (HEADERS
        "include/environment_predictor.h"
	${HEADERS_SHARED}
)

include_directories("include")
add_library ( environment_predictor MODULE ${SOURCES} ${HEADERS})
target_link_libraries(environment_predictor PRIVATE lmscore imaging imaging_detection sensor_utils phoenix_CC2016_service local_course)

