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
#define DISPLAY_OGL 1

// ZED include
#include "SenderRunner.hpp"
#include "GLViewer.hpp"
#include "RelocRW.hpp"
#include "global.hpp"
#include "PracticalSocket.h"
#include "json.hpp"
#include "FusionRunner.hpp"

#include <sl/Camera.hpp>

using namespace sl;

nlohmann::json getJson(sl::FusionMetrics metrics, sl::Bodies& bodies, sl::BODY_FORMAT body_format);
nlohmann::json bodyDataToJson(sl::BodyData body);
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");

/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// -------------------------------- MAIN LOOP ---------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------

FusionRunner fusion;

int main(int argc, char **argv) {

    if (argc != 2) {
        std::cout << "Need a Localization file in input" << std::endl;
        return 1;
    }
    std::string json_config_filename(argv[1]);

    auto z_inputs = sl::readFusionConfigurationFile(json_config_filename, COORD_SYS, UNIT_SYS);

    if (z_inputs.empty()) {
        std::cout << "Empty File " << std::endl;
        return 1;
    }

    std::vector<SenderRunner> senders(z_inputs.size());
    for (int z = 0; z < z_inputs.size(); z++) {
        std::cout << "Try to open ZED " << z_inputs[z].serial_number << ".." << std::endl;
        auto state = senders[z].open(z_inputs[z]);
        if (state)
            std::cout << "ZED " << z_inputs[z].serial_number << " is ready " << std::endl;
        else
            std::cout << "Fail to open ZED " << z_inputs[z].serial_number << std::endl;
    }

    // start sender at the same time (better suited when playing back svos)
    for (int z = 0; z < z_inputs.size(); z++)
        senders[z].start();

    fusion.start(z_inputs);
#if DISPLAY_OGL
    GLViewer viewer;
    viewer.init(argc, argv);
#endif

    bool run = true;

    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------
    std::string servAddress;
    unsigned short servPort;
    UDPSocket sock;

    sock.setMulticastTTL(1);

    servAddress = "230.0.0.1";
    servPort = 20001;

    std::cout << "Sending fused data at " << servAddress << ":" << servPort << std::endl;
    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------

    auto ptr_data = SharedData::getInstance();

    while (run)
    {
        if (ptr_data->bodiesData.mtx.try_lock()) {
#if DISPLAY_OGL
            viewer.updateBodies(ptr_data->bodiesData.bodies, ptr_data->bodiesData.singledata, ptr_data->metrics);
#endif
            if (ptr_data->bodiesData.bodies.is_new)
            {
                try
                {
                    std::string data_to_send = getJson(ptr_data->metrics, ptr_data->bodiesData.bodies, BODY_F).dump();
                    sock.sendTo(data_to_send.data(), data_to_send.size(), servAddress, servPort);
                }
                catch (SocketException& e)
                {

                    cerr << e.what() << endl;
                }
            }
            ptr_data->bodiesData.mtx.unlock();
        }


#if DISPLAY_OGL
        run = viewer.isAvailable();
#endif
        sl::sleep_ms(10);
    }

#if DISPLAY_OGL
    viewer.exit();
#endif

    for (int z = 0; z < z_inputs.size(); z++)
    {
        senders[z].stop();
    }

    fusion.stop();

    return EXIT_SUCCESS;
}



/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------- DATA FORMATTING ------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------

// If the sender encounter NaN values, it sends 0 instead.
nlohmann::json bodyDataToJsonMeter(sl::BodyData body)
{
    nlohmann::json res;

    res["id"] = body.id;
    res["tracking_state"] = body.tracking_state;
    res["action_state"] = body.action_state;

    res["position"] = nlohmann::json::object();
    res["position"]["x"] = isnan(body.position.x) ? 0 : body.position.x;
    res["position"]["y"] = isnan(body.position.y) ? 0 : body.position.y;
    res["position"]["z"] = isnan(body.position.z) ? 0 : body.position.z;

    res["velocity"] = nlohmann::json::object();
    res["velocity"]["x"] = isnan(body.velocity.x) ? 0 : body.velocity.x;
    res["velocity"]["y"] = isnan(body.velocity.y) ? 0 : body.velocity.y;
    res["velocity"]["z"] = isnan(body.velocity.z) ? 0 : body.velocity.z;

    res["confidence"] = isnan(body.confidence) ? 0 : body.confidence;

    res["bounding_box"] = nlohmann::json::array();
    for (auto& i : body.bounding_box) {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x;
        e["y"] = isnan(i.y) ? 0 : i.y;
        e["z"] = isnan(i.z) ? 0 : i.z;
        res["bounding_box"].push_back(e);
    }

    res["dimensions"] = nlohmann::json::object();
    res["dimensions"]["x"] = isnan(body.dimensions.x) ? 0 : body.dimensions.x;
    res["dimensions"]["y"] = isnan(body.dimensions.y) ? 0 : body.dimensions.y;
    res["dimensions"]["z"] = isnan(body.dimensions.z) ? 0 : body.dimensions.z;

    res["keypoint"] = nlohmann::json::array();
    for (auto& i : body.keypoint) {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x;
        e["y"] = isnan(i.y) ? 0 : i.y;
        e["z"] = isnan(i.z) ? 0 : i.z;
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
        e["x"] = isnan(i.x) ? 0 : i.x;
        e["y"] = isnan(i.y) ? 0 : i.y;
        e["z"] = isnan(i.z) ? 0 : i.z;
        res["local_position_per_joint"].push_back(e);
    }
    res["local_orientation_per_joint"] = nlohmann::json::array();
    for (auto& i : body.local_orientation_per_joint)
    {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x;
        e["y"] = isnan(i.y) ? 0 : i.y;
        e["z"] = isnan(i.z) ? 0 : i.z;
        e["w"] = isnan(i.w) ? 1 : i.w;
        res["local_orientation_per_joint"].push_back(e);
        if(isnan(i.x) || isnan(i.y) || isnan(i.z) || isnan(i.w)) { std::cout << "Nan value in quat." << std::endl; }
    }
    res["global_root_orientation"] = nlohmann::json::object();
    res["global_root_orientation"]["x"] = isnan(body.global_root_orientation.x) ? 0 : body.global_root_orientation.x;
    res["global_root_orientation"]["y"] = isnan(body.global_root_orientation.y) ? 0 : body.global_root_orientation.y;
    res["global_root_orientation"]["z"] = isnan(body.global_root_orientation.z) ? 0 : body.global_root_orientation.z;
    res["global_root_orientation"]["w"] = isnan(body.global_root_orientation.w) ? 0 : body.global_root_orientation.w;
    return res;
}


// Create the json sent to the clients
nlohmann::json getJson(sl::FusionMetrics metrics, sl::Bodies& bodies, sl::BODY_FORMAT body_format)
{
    nlohmann::json j;

    nlohmann::json bodyData;
    nlohmann::json fusionMetricsData;
    nlohmann::json singleMetricsData;

    fusionMetricsData["mean_camera_fused"] = metrics.mean_camera_fused;
    fusionMetricsData["mean_stdev_between_camera"] = metrics.mean_stdev_between_camera;

    for (auto& cam : fusion.getCameras())
    {

        singleMetricsData["sn"] = cam.sn;
        singleMetricsData["received_fps"] = metrics.camera_individual_stats[cam.sn].received_fps;
        singleMetricsData["received_latency"] = metrics.camera_individual_stats[cam.sn].received_latency;
        singleMetricsData["synced_latency"] = metrics.camera_individual_stats[cam.sn].synced_latency;
        singleMetricsData["is_present"] = metrics.camera_individual_stats[cam.sn].is_present;
        singleMetricsData["ratio_detection"] = metrics.camera_individual_stats[cam.sn].ratio_detection;
        singleMetricsData["delta_ts"] = metrics.camera_individual_stats[cam.sn].delta_ts;

        fusionMetricsData["camera_individual_stats"].push_back(singleMetricsData);
    }

    bodyData["body_format"] = body_format;

    bodyData["is_new"] = (int)bodies.is_new;
    bodyData["is_tracked"] = (int)bodies.is_tracked;
    bodyData["timestamp"] = bodies.timestamp.data_ns;

    bodyData["nb_object"] = bodies.body_list.size();
    bodyData["body_list"] = nlohmann::json::array();

    for (auto& body : bodies.body_list)
    {
        bodyData["body_list"].push_back(bodyDataToJsonMeter(body));
        if (body.local_orientation_per_joint.size() == 0)
        {
            std::cout << "Detected body[" << body.id << "] has empty jointsOrientation, with tracking state : " << body.tracking_state << std::endl;
        }
    }

    j["fusionMetrics"] = fusionMetricsData;
    j["bodies"] = bodyData;

    return j;
}

/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------- MISC & MONO-CAM ------------------------------
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
    //res["position_covariance"] = nlohmann::json::array();
    //for (auto& i : body.position_covariance) 
    // {
    //    res["position_covariance"].push_back(i);
    //}
    //res["bounding_box_2d"] = nlohmann::json::array();
    //for (auto& i : body.bounding_box_2d) 
    // {
    //    nlohmann::json e;
    //    e["x"] = i.x;
    //    e["y"] = i.y;
    //    res["bounding_box_2d"].push_back(e);
    //}
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
    //res["keypoint_2d"] = nlohmann::json::array();
    //for (auto& i : body.keypoint_2d) 
    // {
    //    nlohmann::json e;
    //    e["x"] = i.x;
    //    e["y"] = i.y;
    //    res["keypoint_2d"].push_back(e);
    //}
    res["keypoint"] = nlohmann::json::array();
    for (auto& i : body.keypoint) {
        nlohmann::json e;
        e["x"] = isnan(i.x) ? 0 : i.x / 1000;
        e["y"] = isnan(i.y) ? 0 : i.y / 1000;
        e["z"] = isnan(i.z) ? 0 : i.z / 1000;
        res["keypoint"].push_back(e);
    }
    std::cout << "id [" << body.id << "] hips: " << body.keypoint[0] << std::endl;
    //res["head_bounding_box_2d"] = nlohmann::json::array();
    //for (auto& i : body.head_bounding_box_2d) 
    // {
    //    nlohmann::json e;
    //    e["x"] = i.x;
    //    e["y"] = i.y;
    //    res["head_bounding_box_2d"].push_back(e);
    //}
    //res["head_bounding_box"] = nlohmann::json::array();
    //for (auto& i : body.head_bounding_box)
    //  {
    //    nlohmann::json e;
    //    e["x"] = i.x;
    //    e["y"] = i.y;
    //    e["z"] = i.z;
    //    res["head_bounding_box"].push_back(e);
    //}
    //res["head_position"] = nlohmann::json::object();
    //res["head_position"]["x"] = body.head_position.x;
    //res["head_position"]["y"] = body.head_position.y;
    //res["head_position"]["z"] = body.head_position.z;
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