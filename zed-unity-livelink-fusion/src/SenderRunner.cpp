#include "SenderRunner.hpp"

SenderRunner::SenderRunner() : running(false) {
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.camera_fps = 30;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.sdk_verbose = 6;
    init_params.svo_real_time_mode = true;
}

SenderRunner::~SenderRunner() {
    zed.close();
}

bool SenderRunner::open(sl::InputType input, sl::BODY_FORMAT body_format) {
    // already running
    if (runner.joinable())
        return false;

    init_params.input = input;
    auto state = zed.open(init_params);
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    // in most cases in body tracking setup, the cameras are static
    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = true;
    state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    // define the body tracking parameters, as the fusion can does the tracking and fitting you don't need to enable them here, unless you need it for your app
    sl::BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.detection_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
    body_tracking_parameters.body_format = body_format;
    body_tracking_parameters.enable_body_fitting = false;
    body_tracking_parameters.enable_tracking = false;
    state = zed.enableBodyTracking(body_tracking_parameters);
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    return true;
}


void SenderRunner::start()
{

    if (zed.isOpened()) {
        running = true;
        // the camera should stream its data so the fusion can subscibe to it to gather the detected body and others metadata needed for the process.
        zed.startPublishing();
        // the thread can start to process the camera grab in background
        runner = std::thread(&SenderRunner::work, this);
    }
}

void SenderRunner::stop() 
{
    running = false;
    if (runner.joinable())
        runner.join();
    zed.close();
}

void SenderRunner::work() 
{
    sl::Bodies bodies;
    sl::BodyTrackingRuntimeParameters body_runtime_parameters;
    body_runtime_parameters.detection_confidence_threshold = 40;

    // in this sample we use a dummy thread to process the ZED data.
    // you can replace it by your own application and use the ZED like you use to, retrieve its images, depth, sensors data and so on.
    // as long as you call the grab function and the retrieveBodies (wich run the detection) the camera will be able to seamlessly transmit the data to the fusion module.
    while (running)
    {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS)
        {

            /*
            Your App
            */

            // just be sure to run the bodies detection
            zed.retrieveBodies(bodies, body_runtime_parameters);
        }
    }
}
