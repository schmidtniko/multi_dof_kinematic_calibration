#include <ceres/ceres.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include "multi_dof_kinematic_calibration/PtuCalibrationProject.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


//------------------------------------------------------------------------------------------------------------
namespace po = boost::program_options;
po::variables_map loadParameters(int argc, char* argv[])
{
    po::options_description options("Allowed options");
    po::options_description fileOptions("Options for the config file");
    options.add_options()("help,?", "produces this help message")(
        "project_path", po::value<std::string>()->required(), "Path to project to be processed");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    if (vm.count("help"))
    {
        std::cout << options << std::endl;
        std::cout << fileOptions << std::endl;
        exit(0);
    }

    po::notify(vm);

    return vm;
}
//------------------------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    try
    {
        boost::program_options::variables_map vm = loadParameters(argc, argv);

        const boost::filesystem::path project_path = vm["project_path"].as<std::string>();

        PtuCalibrationProject proj;
        proj.processFolder(project_path.string());

    }
    catch (const std::exception& ex)
    {
        std::cout << "An exception occurred: " << ex.what() << std::endl;
    }
    return 0;
}