#include "SenderRunner.hpp"

#include "RelocRW.hpp"

#include "global.hpp"

SenderRunner::SenderRunner() : running(false) {
    init_params.coordinate_units = UNIT_SYS;
    init_params.coordinate_system = COORD_SYS;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.camera_fps = 30;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.sdk_verbose = 6;
    init_params.svo_real_time_mode = true;
}

SenderRunner::~SenderRunner() {
    zed.close();
}

bool SenderRunner::open(sl::FusionConfiguration z_input) {
    // already running
    if (runner.joinable()) return false;

    zed_config = z_input;
    init_params.input = zed_config.input_type;
    auto state = zed.open(init_params);
    if (state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error open Camera " << state << std::endl;
        return false;
    }

    sl::PositionalTrackingParameters ptp;
    ptp.set_as_static = true;

    state = zed.enablePositionalTracking(ptp);    
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error enable Positional Tracking" << state << std::endl;
        return false;
    }

    sl::BodyTrackingParameters btp;
    btp.detection_model = BODY_M;
    btp.body_format = BODY_F;
    btp.enable_body_fitting = false;
    btp.enable_tracking = false;
    state = zed.enableBodyTracking(btp);

    if (state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error enable Body Tracking " << state << std::endl;
        return false;
    }
    return true;
}


void SenderRunner::start() {
    if (zed.isOpened())
    {
        running = true;
        zed.startPublishing(zed_config.communication_parameters);
        runner = std::thread(&SenderRunner::work, this);
    }
}

void SenderRunner::stop() {
    running = false;
    if (runner.joinable()) runner.join();
}

void SenderRunner::startSVOrecording(std::string fileName) {
    sl::RecordingParameters rp;
    fileName += "_SN" + std::to_string(zed_config.serial_number)+".svo";
    rp.video_filename.set(fileName.c_str());
    rp.compression_mode = sl::SVO_COMPRESSION_MODE::H265;
    zed.enableRecording(rp);
}

void SenderRunner::stopSVOrecording() {
    zed.disableRecording();
}

void SenderRunner::work() {
    running = true;

    sl::BodyTrackingRuntimeParameters rtp = sl::BodyTrackingRuntimeParameters();
    rtp.detection_confidence_threshold = 40;
    rtp.minimum_keypoints_threshold = -1;

    sl::Bodies local_objs;
    int i = 0;
    while (running) {
        auto err = zed.grab();
        if (err == sl::ERROR_CODE::SUCCESS) {
            // run Detection
            zed.retrieveBodies(local_objs, rtp);
        }
        else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            zed.setSVOPosition(0);
        }
    }
}