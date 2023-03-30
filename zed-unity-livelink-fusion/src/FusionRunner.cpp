#include "FusionRunner.hpp"

#include "RelocRW.hpp"


#include "global.hpp"

FusionRunner::FusionRunner() : running(false) {
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
    // This parameters has to be set to true. Do not change it.
    init_params.output_performance_metrics = true;
    init_params.verbose = true;
}

FusionRunner::~FusionRunner() {
    fusion.close();
}

std::vector<sl::CameraIdentifier> FusionRunner::getCameras()
{
    return cameras;
}

bool FusionRunner::start(std::vector<sl::FusionConfiguration>& z_inputs) {
    // already running
    if (runner.joinable()) return false;

    fusion.init(init_params);

    // read the Localization file to know wich camera (Sender) should be connected
    for (auto& it : z_inputs) {
        sl::CameraIdentifier uuid;
        uuid.sn = it.serial_number;
        auto state = fusion.subscribe(uuid, it.communication_parameters, it.pose);
        if (state != sl::FUSION_ERROR_CODE::SUCCESS)
            std::cout << "Unable to subscribe to " << std::to_string(uuid.sn) << " . " << state << std::endl;
        else
            cameras.push_back(uuid);
    }

    if (cameras.empty()) {
        std::cout << "No Camera connected.\n\tQUIT." << std::endl;
        return false;
    }

    //Make sure to enable the same 'detection_model' as the one defined in the Sender
    sl::BodyTrackingFusionParameters BT_fusion_init_params;
    BT_fusion_init_params.enable_tracking = true; // always required
    BT_fusion_init_params.enable_body_fitting = true; // skeletons will looks more natural but requires more computations

    fusion.enableBodyTracking(BT_fusion_init_params);

    runner = std::thread(&FusionRunner::work, this);
    return true;
}

void FusionRunner::stop() {
    running = false;
    if (runner.joinable()) runner.join();
}

void FusionRunner::work() {
    running = true;

    // define fusion behavior    
    sl::BodyTrackingFusionRuntimeParameters bt_rt_params;
    bt_rt_params.skeleton_minimum_allowed_keypoints = -1;
    bt_rt_params.skeleton_minimum_allowed_camera = -1;
    auto ptr_data = SharedData::getInstance();

    while (running) {
        if (fusion.process() == sl::FUSION_ERROR_CODE::SUCCESS) {
            {
                const std::lock_guard<std::mutex> lock_m(ptr_data->bodiesData.mtx);
                // Retrieve detected objects
                fusion.retrieveBodies(ptr_data->bodiesData.bodies, bt_rt_params);
                // for debug, you can retrieve the data send by each camera, as well as communication and process stat just to make sure everything is okay
                //for (auto& id : cameras)
                //    fusion.retrieveBodies(ptr_data->bodiesData.singledata[id], bt_rt_params, id);
            }

            if (init_params.output_performance_metrics)
                fusion.getProcessMetrics(ptr_data->metrics);
        }
    }
}