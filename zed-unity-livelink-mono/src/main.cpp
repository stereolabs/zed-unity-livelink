///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// OpenGL Viewer(!=0) or no viewer (0)
#define DISPLAY_OGL 0

// ZED include
#include "GLViewer.hpp"
#include "PracticalSocket.h"
#include "json.hpp"

#include <sl/Camera.hpp>

using namespace sl;

nlohmann::json getJson(sl::Camera& pcamera, sl::Bodies& bodies, sl::BODY_FORMAT body_format);
nlohmann::json getJson(sl::Camera& pcamera, sl::Bodies& bodies, int id, sl::BODY_FORMAT body_format);

nlohmann::json bodyDataToJson(sl::BodyData body);
void parseArgsMonoCam(int argc, char** argv, InitParameters& param);
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");


/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// -------------------------------- MAIN LOOP ---------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------

int main(int argc, char **argv) {

    Camera zed;
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD720;
    init_parameters.camera_fps = 30;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Y_UP; // Coordinate system of Unity.
    init_parameters.svo_real_time_mode = true;    

    // Parse the command line arguments (see function definition)
    parseArgsMonoCam(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    // Enable Positional tracking (mandatory for body tracking) -------------------------------------------------------
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_floor_as_origin = true;
    // If the camera is static, uncomment the following line to have better performance.
    // positional_tracking_parameters.set_as_static = true;

    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Positional Tracking", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    // Enable the Body tracking module -------------------------------------------------------------------------------------
    BodyTrackingParameters body_tracker_params;
    body_tracker_params.enable_tracking = true; // track people across grabs
    body_tracker_params.enable_body_fitting = true; // smooth skeletons moves
    body_tracker_params.body_format = sl::BODY_FORMAT::BODY_38;
    body_tracker_params.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;
    returned_state = zed.enableBodyTracking(body_tracker_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Body Tracking", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }


#if DISPLAY_OGL
    GLViewer viewer;
    viewer.init(argc, argv);
#endif

    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    // Configure body tracking runtime parameters
    BodyTrackingRuntimeParameters body_tracker_parameters_rt;
    body_tracker_parameters_rt.detection_confidence_threshold = 40;
    body_tracker_parameters_rt.minimum_keypoints_threshold = 10;

    // Create ZED Bodies filled in the main loop
    Bodies bodies;

    unsigned int serial_num = 0;
    bool run = true;

    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------
    std::string servAddress;
    unsigned short servPort;
    UDPSocket sock;

    //sock.setMulticastTTL(1);

    servAddress = "127.0.0.1";
    servPort = 20001;

    std::cout << "Sending fused data at " << servAddress << ":" << servPort << std::endl;
    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------

    RuntimeParameters rt_params = new RuntimeParameters();
    rt_params.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

    std::cout << "Sending Mono-Camera data at " << servAddress << ":" << servPort << std::endl;

    while (run)
    {
        auto err = zed.grab(rt_params);
        if (err == ERROR_CODE::SUCCESS) 
        {
            // Retrieve Detected Human Bodies
            zed.retrieveBodies(bodies, body_tracker_parameters_rt);
#if DISPLAY_OGL
            //Update GL View
            viewer.updateData(bodies, cam_pose.pose_data);
#endif

            if (bodies.is_new) {
                try
                {
                    //std::cout << "Size of data to send " << data_to_send.size() << std::endl;
                    for (int i = 0; i < bodies.body_list.size(); i++)
                    {
                        std::string data_to_send = getJson(zed, bodies, i, body_tracker_params.body_format).dump();
                        sock.sendTo(data_to_send.data(), data_to_send.size(), servAddress, servPort);

                        sl::sleep_us(100);
                    }
                }
                catch (SocketException& e)
                {
                    cerr << e.what() << endl;
                    //exit(1);
                }
            }
        }
        else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            zed.setSVOPosition(0);
        }
        else
        {
            print("error grab", returned_state, "\nExit program.");
        }

#if DISPLAY_OGL
        run = viewer.isAvailable();
#endif

        sl::sleep_ms(10);
    }

#if DISPLAY_OGL
    viewer.exit();
#endif

    // Release Bodies
    bodies.body_list.clear();

    // Disable modules
    zed.disableBodyTracking();
    zed.disablePositionalTracking();
    zed.close();

    return EXIT_SUCCESS;
}



/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------- DATA FORMATTING ------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------


void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error]";
    cout << " " << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

// Create the json sent to the clients
nlohmann::json getJson(sl::Camera& pcamera, sl::Bodies& bodies, sl::BODY_FORMAT body_format)
{
    Pose camp;
    pcamera.getPosition(camp);
    ///std::cout << "camera position : " << camp.getTranslation() << std::endl;

    nlohmann::json j;

    nlohmann::json bodyData;

    bodyData["body_format"] = body_format;
    bodyData["is_new"] = (int)bodies.is_new;
    bodyData["is_tracked"] = (int)bodies.is_tracked;
    bodyData["timestamp"] = bodies.timestamp.data_ns;

    bodyData["nb_object"] = bodies.body_list.size();
    bodyData["body_list"] = nlohmann::json::array();

    for (auto& body : bodies.body_list)
    {
        bodyData["body_list"].push_back(bodyDataToJson(body));
    }

    j["bodies"] = bodyData;

    return j;
}


// send one skeleton at a time
nlohmann::json getJson(sl::Camera& pcamera, sl::Bodies& bodies, int id, sl::BODY_FORMAT body_format)
{
    Pose camp;
    pcamera.getPosition(camp);
    ///std::cout << "camera position : " << camp.getTranslation() << std::endl;

    nlohmann::json j;

    nlohmann::json bodyData;

    bodyData["body_format"] = body_format;
    bodyData["is_new"] = (int)bodies.is_new;
    bodyData["is_tracked"] = (int)bodies.is_tracked;
    bodyData["timestamp"] = bodies.timestamp.data_ns;

    bodyData["nb_object"] = bodies.body_list.size();
    bodyData["body_list"] = nlohmann::json::array();


    if (id < bodies.body_list.size())
    {
        bodyData["body_list"].push_back(bodyDataToJson(bodies.body_list[id]));
    }

    j["bodies"] = bodyData;

    return j;
}

// If the sender encounter NaN values, it sends 0 instead.
nlohmann::json bodyDataToJson(sl::BodyData body)
{
    nlohmann::json res;

    res["id"] = body.id;
    //res["unique_object_id"] = body.unique_object_id.get();
    res["tracking_state"] = body.tracking_state;
    res["action_state"] = body.action_state;
    res["position"] = nlohmann::json::object();
    res["position"]["x"] = isnan(body.position.x) ? 0 : body.position.x / 1000;
    res["position"]["y"] = isnan(body.position.y) ? 0 : body.position.y / 1000;
    res["position"]["z"] = isnan(body.position.z) ? 0 : body.position.z / 1000;

    res["velocity"] = nlohmann::json::object();
    res["velocity"]["x"] = isnan(body.velocity.x) ? 0 : body.velocity.x / 1000;
    res["velocity"]["y"] = isnan(body.velocity.y) ? 0 : body.velocity.y / 1000;
    res["velocity"]["z"] = isnan(body.velocity.z) ? 0 : body.velocity.z / 1000;

    res["confidence"] = isnan(body.confidence) ? 0 : body.confidence;
    res["bounding_box"] = nlohmann::json::array();
    for (auto& i : body.bounding_box) {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x / 1000;
        e["y"] = isnan(i.y) ? 0 : i.y / 1000;
        e["z"] = isnan(i.z) ? 0 : i.z / 1000;
        res["bounding_box"].push_back(e);
    }
    res["dimensions"] = nlohmann::json::object();
    res["dimensions"]["x"] = isnan(body.dimensions.x) ? 0 : body.dimensions.x / 1000;
    res["dimensions"]["y"] = isnan(body.dimensions.y) ? 0 : body.dimensions.y / 1000;
    res["dimensions"]["z"] = isnan(body.dimensions.z) ? 0 : body.dimensions.z / 1000;

    res["keypoint"] = nlohmann::json::array();
    for (auto& i : body.keypoint) {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x / 1000;
        e["y"] = isnan(i.y) ? 0 : i.y / 1000;
        e["z"] = isnan(i.z) ? 0 : i.z / 1000;
        res["keypoint"].push_back(e);
    }

    res["keypoint_confidence"] = nlohmann::json::array();
    for (auto& i : body.keypoint_confidence)
    {
        res["keypoint_confidence"].push_back(isnan(i) ? 0 : i);
    }
    res["local_position_per_joint"] = nlohmann::json::array();
    for (auto& i : body.local_position_per_joint)
    {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x / 1000;
        e["y"] = isnan(i.y) ? 0 : i.y / 1000;
        e["z"] = isnan(i.z) ? 0 : i.z / 1000;
        res["local_position_per_joint"].push_back(e);
    }
    res["local_orientation_per_joint"] = nlohmann::json::array();
    for (auto& i : body.local_orientation_per_joint)
    {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 42 : i.x;
        e["y"] = isnan(i.y) ? 42 : i.y;
        e["z"] = isnan(i.z) ? 42 : i.z;
        e["w"] = isnan(i.w) ? 42 : i.w;
        res["local_orientation_per_joint"].push_back(e);
    }
    res["global_root_orientation"] = nlohmann::json::object();
    res["global_root_orientation"]["x"] = isnan(body.global_root_orientation.x) ? 0 : body.global_root_orientation.x;
    res["global_root_orientation"]["y"] = isnan(body.global_root_orientation.y) ? 0 : body.global_root_orientation.y;
    res["global_root_orientation"]["z"] = isnan(body.global_root_orientation.z) ? 0 : body.global_root_orientation.z;
    res["global_root_orientation"]["w"] = isnan(body.global_root_orientation.w) ? 0 : body.global_root_orientation.w;
    return res;
}

// Parse command line arguments for mono-camera.
void parseArgsMonoCam(int argc, char** argv, InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    }
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        }
        else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }
        else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        }
        else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }
        else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
}