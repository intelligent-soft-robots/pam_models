#pragma once

#include <iostream>
#include <string>

#include "json_helper/json_helper.hpp"

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

    /**
     * Generates json file in given file path with given test value.
     *
     * @param file_path Absolute file path for generation of json file
     * @param test_value Test value which is loaded into generated json file
     * @remark Main purpose of this function is unit testing
     */
    void generate_test_json(std::string file_path, double test_value);

    /**
     * Loads test value from json file specified by given file path.
     *
     * @param file_path Absolute file path of json file to be loaded
     * @return Test value stored in json file
     * @remark Main purpose of this function is unit testing
     */
    double load_test_json(std::string file_path);
};

}  // namespace hill

}  // namespace pam_models