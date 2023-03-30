#ifndef __ZED_RELOC_RW_HDR__
#define __ZED_RELOC_RW_HDR__

#include "json.hpp"
#include <sl/Camera.hpp>

struct ZED_INPUT {
    sl::InputType input;
    sl::Transform pose;
    int serial;
};

namespace reloc{

    inline
    void sliptIP(std::string ipAdd, std::string &ip, unsigned short &port){
        std::size_t found_port = ipAdd.find_last_of(":");
        ip = ipAdd;
        port = 30000;
        if (found_port != std::string::npos) {
            ip = ipAdd.substr(0, found_port);
            port = std::atoi(ipAdd.substr(found_port + 1).c_str());
        }
    }

    inline 
    sl::InputType getInput(sl::InputType::INPUT_TYPE type, std::string conf){
        sl::InputType input;
        switch(type){
            case sl::InputType::INPUT_TYPE::USB_ID: input.setFromCameraID(atoi(conf.c_str()), sl::BUS_TYPE::USB); break;
            case sl::InputType::INPUT_TYPE::GMSL_ID: input.setFromCameraID(atoi(conf.c_str()), sl::BUS_TYPE::GMSL); break;
            case sl::InputType::INPUT_TYPE::USB_SERIAL:
            case sl::InputType::INPUT_TYPE::GMSL_SERIAL:
            input.setFromSerialNumber(atoi(conf.c_str()));
            break;
            case sl::InputType::INPUT_TYPE::STREAM:{
                std::string IP_add;
                unsigned short port;
                sliptIP(conf, IP_add, port);
                input.setFromStream(IP_add.c_str(), port);
            } break;
            case sl::InputType::INPUT_TYPE::SVO_FILE: input.setFromSVOFile(sl::String(conf.c_str())); break;
        }
        return input;
    }

    inline 
    ZED_INPUT read(nlohmann::json &injson, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit){
        ZED_INPUT z_input;
        float ratio_ = sl::getUnitScale(sl::UNIT::METER, unit);
        auto tr_im2user = sl::getCoordinateTransformConversion4f(sl::COORDINATE_SYSTEM::IMAGE, coord_sys);
        
        sl::Transform pose_im;
        auto j_rot = injson["world"]["rotation"];
        pose_im.setRotationVector(sl::float3(j_rot[0].get<float>(), j_rot[1].get<float>(), j_rot[2].get<float>()));
        auto j_tr = injson["world"]["translation"];
        pose_im.setTranslation(sl::float3(j_tr[0].get<float>() * ratio_, j_tr[1].get<float>() * ratio_, j_tr[2].get<float>() * ratio_));
        
        z_input.pose = tr_im2user * pose_im * sl::Matrix4f::inverse(tr_im2user);

        auto i_type = injson["input"]["zed"]["type"].get<sl::InputType::INPUT_TYPE>();
        std::string i_conf = injson["input"]["zed"]["configuration"].get<std::string>();        
        z_input.input = getInput(i_type, i_conf);
        return z_input;
    }

    inline 
    nlohmann::json write(ZED_INPUT &it, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit, int id){
        nlohmann::json current_zed;
        auto tr_im2user = sl::getCoordinateTransformConversion4f(coord_sys, sl::COORDINATE_SYSTEM::IMAGE);
        sl::Transform pose_im = tr_im2user * it.pose * sl::Matrix4f::inverse(tr_im2user);
        auto vec_r = pose_im.getRotationVector();
        current_zed["world"]["rotation"] = { vec_r.x, vec_r.y, vec_r.z };
        float ratio_ = sl::getUnitScale(unit, sl::UNIT::METER);
        auto tr = pose_im.getTranslation() * ratio_;
        current_zed["world"]["translation"] = { tr.x, tr.y, tr.z };

        current_zed["input"]["zed"]["type"] = it.input.getType();
        current_zed["input"]["zed"]["configuration"] = std::string(it.input.getConfiguration().c_str());

        std::string ip;
        unsigned short port;
        if(it.input.getType() == sl::InputType::INPUT_TYPE::STREAM)
            sliptIP(it.input.getConfiguration().c_str(), ip, port);
        else
            ip = "127.0.0.1"; // localhost        
        current_zed["input"]["fusion"]["type"] = "network";
        current_zed["input"]["fusion"]["configuration"]["port"] = 30000 + (id *2);
        current_zed["input"]["fusion"]["configuration"]["ip"] = ip;

        return current_zed;
    }
}

inline 
 std::vector<ZED_INPUT> readZEDLocation(std::string file, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit){
     std::vector<ZED_INPUT> zeds_input;
    nlohmann::json json_file;
    std::ifstream in(file);
    if(in.is_open()){
        in >> json_file;
        in.close();
        for (auto& it : json_file.get<nlohmann::json::object_t>()){
            ZED_INPUT zed_i = reloc::read(it.second, coord_sys, unit);
            zed_i.serial = atoi(it.first.c_str());
            zeds_input.push_back(zed_i);
        }
    }
    return zeds_input;
}

inline 
void saveZEDLocations(std::string fileName, std::vector<ZED_INPUT> &zed_inputs, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit){
    nlohmann::json jsonfile;
    int id = 0;
    for(auto  &it: zed_inputs)
        jsonfile[std::to_string(it.serial)] = reloc::write(it, coord_sys, unit, id++);    
    std::ofstream file_cam(fileName);
    file_cam << std::setw(4) << jsonfile << std::endl;
    file_cam.close();
}

#endif //__ZED_RELOC_RW_HDR__
