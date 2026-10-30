#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;
    // 感知版 walk 的高程图发布（见 height_map_publisher.h），可选项，缺省关闭
    int enable_height_map = 0;
    std::string height_map_topic = "rt/perceptive/height_map";
    std::string height_map_body = "torso_link";
    double height_map_rate = 10.0;
    // MuJoCo 查看器里的感知叠加显示（见 perception_overlay.h），M / C 键开关
    int enable_perception_overlay = 1;
    int overlay_show_height_map = 1;
    int overlay_show_camera = 0;
    double overlay_rate = 20.0;
    int overlay_camera_cols = 24;
    int overlay_camera_rows = 14;
    double overlay_camera_range = 4.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["enable_height_map"]) enable_height_map = cfg["enable_height_map"].as<int>();
            if (cfg["height_map_topic"]) height_map_topic = cfg["height_map_topic"].as<std::string>();
            if (cfg["height_map_body"]) height_map_body = cfg["height_map_body"].as<std::string>();
            if (cfg["height_map_rate"]) height_map_rate = cfg["height_map_rate"].as<double>();
            if (cfg["enable_perception_overlay"]) enable_perception_overlay = cfg["enable_perception_overlay"].as<int>();
            if (cfg["overlay_show_height_map"]) overlay_show_height_map = cfg["overlay_show_height_map"].as<int>();
            if (cfg["overlay_show_camera"]) overlay_show_camera = cfg["overlay_show_camera"].as<int>();
            if (cfg["overlay_rate"]) overlay_rate = cfg["overlay_rate"].as<double>();
            if (cfg["overlay_camera_cols"]) overlay_camera_cols = cfg["overlay_camera_cols"].as<int>();
            if (cfg["overlay_camera_rows"]) overlay_camera_rows = cfg["overlay_camera_rows"].as<int>();
            if (cfg["overlay_camera_range"]) overlay_camera_range = cfg["overlay_camera_range"].as<double>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}