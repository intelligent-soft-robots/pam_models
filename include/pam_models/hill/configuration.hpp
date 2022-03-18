#pragma once

#include <iostream>
#include <string>

#include "json_helper/json_helper.hpp"
#include "pam_configuration/pam_configuration.hpp"

// parent folder will be ~/.mpi-is/pam
// or /opt/mpi-is/pam
#define HILL_JSON_RELATIVE_PATH "pam_models/hill.json"

namespace pam_models
{
namespace hill
{
/**
 * @brief Configuration for loading parameters from configuration files
 *
 * The configuration class loads and provides muscle parameters from
 * json files
 */
class Configuration
{
public:
    /**
     * Current muscle length from MuJoCo
     */
    double length;

    /**
     * Configuration parameters for contractile element
     */
    std::map<std::string, double> CE_parameter_storage;

    /**
     * Configuration parameters for parallel elastic element
     */
    std::map<std::string, double> PEE_parameter_storage;

    /**
     * Configuration parameters for serial damping element
     */
    std::map<std::string, double> SDE_parameter_storage;

    /**
     * Configuration parameters for serial elastic element
     */
    std::map<std::string, double> SEE_parameter_storage;

    /**
     * Loads configuration from json file into configuration manager.
     *
     * @param file_path Absolute file path to json configuration file
     */
    void load_from_json(std::string file_path);

    /**
     * Loads configuration from default json file into configuration manager.
     */
    void load_from_default_json();

public:
    /**
     * Returns the path to the default json configuration file,
     * i.e. either ~/.mpi-is/pam/pam_models/hill.json or
     * /opt/mpi-is/pam/pam_models/hill.json (see the pam_configuration
     * package)
     */
    static std::filesystem::path get_default_json_file();
};

}  // namespace hill

}  // namespace pam_models
