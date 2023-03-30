#ifndef  __GLOBAL_HDR__
#define __GLOBAL_HDR__

#include <sl/Fusion.hpp>

const sl::COORDINATE_SYSTEM COORD_SYS = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
const sl::UNIT UNIT_SYS = sl::UNIT::METER;

static const sl::BODY_TRACKING_MODEL BODY_M = sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
static const sl::BODY_FORMAT BODY_F = sl::BODY_FORMAT::BODY_38;

struct BodiesData {
    std::map<sl::CameraIdentifier, sl::Bodies> singledata;
    sl::Bodies bodies;
    std::mutex mtx;
};


class SharedData {
public:

    static SharedData* getInstance() {
        static SharedData it;
        return &it;
    }

    BodiesData bodiesData;
    sl::FusionMetrics metrics;
};

#endif // ! __GLOBAL_HDR__
