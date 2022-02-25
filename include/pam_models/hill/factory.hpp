#pragma once

#include <iostream>
#include <string>

#include "pam_models/hill/configuration.hpp"
#include "pam_models/hill/muscle.hpp"

namespace pam_models
{
namespace hill
{
/**
 * Generates Configuration object from given file_path
 *
 * @param file_path Full file path specifying location of configuration file
 * @return Configuration object
 */
Configuration generate_configuration_from_path(std::string file_path);

/**
 * @brief Muscle class generator with parameter from configuration file
 *
 * Generates Hill-type muscle class object with parameters obtained
 * from a json configuration file specified in the given file path.
 *
 * @param file_path Full file path specifying location of configuration file
 * @param a_init Initial muscle activation with minimally activated muscle
 * @param l_MTC_change_init Initial length of muscle-tendon-complex (MTC)
 * @return Hill-type muscle-class object
 */
Muscle from_json(std::string file_path,
                 double a_init,
                 double l_MTC_change_init);

/**
 * @brief Default muscle class generator
 *
 * Generates Hill-type muscle class object with default parameters obtained
 * from default json configuration.
 *
 * @param a_init Initial muscle activation with minimally activated muscle
 * @param l_MTC_change_init Initial length of muscle-tendon-complex (MTC)
 * @return Default hill-type muscle-class object
 *
 * @remark Default parameters are by default stored in
 *         /opt/mpi-is/pam_models/hill.json
 */
Muscle from_default_json(double a_init, double l_MTC_change_init);

/**
 * @brief Contractile element (CE) generator
 *
 * Generates contractile element (CE) with parameters obtained from a json
 * configuration file specified in the given file path.
 *
 * @param file_path full file path specifying location of configuration file
 * @return Contractile element object
 */
ContractileElement generate_CE(std::string file_path);

/**
 * @brief Parallel elastic element (PEE) generator
 *
 * Generates parallel elastic element (PEE) with parameters obtained from a json
 * configuration file specified in the given file path.
 *
 * @param file_path Full file path specifying location of configuration file
 * @return Parallel elastic element object
 */
ParallelElasticElement generate_PEE(std::string file_path);

/**
 * @brief Serial damping element (SDE) generator
 *
 * Generates serial damping element (SDE) with parameters obtained from a json
 * configuration file specified in the given file path.
 *
 * @param file_path Full file path specifying location of configuration file
 * @return Serial damping element object
 */
SerialDampingElement generate_SDE(std::string file_path);

/**
 * @brief Serial elastic element (SEE) generator
 *
 * Generates serial elastic element (SEE) with parameters obtained from a json
 * configuration file specified in the given file path.
 *
 * @param file_path Full file path specifying location of configuration file
 * @return Serial elastic element object
 */
SerialElasticElement generate_SEE(std::string file_path);

}  // namespace hill

}  // namespace pam_models