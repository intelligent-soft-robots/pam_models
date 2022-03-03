#include "pam_models/hill/configuration.hpp"

namespace pam_models
{
namespace hill
{
void Configuration::load_from_json(std::string file_path)
{
    json_helper::Jsonhelper jh;

    try
    {
        jh.parse(file_path);
    }
    catch (...)
    {
        std::stringstream ss;
        ss << "Failed to read JSON file " << file_path << "\n";
        throw std::runtime_error(ss.str());
    }

    // contractile element parameters
    CE_parameter_storage["f_max"] = jh.j["contractile"]["f_max"].get<double>();
    CE_parameter_storage["l_CEopt"] =
        jh.j["contractile"]["l_CEopt"].get<double>();
    CE_parameter_storage["delta_w_limb_desc"] =
        jh.j["contractile"]["delta_w_limb_desc"].get<double>();
    CE_parameter_storage["delta_w_limb_asc"] =
        jh.j["contractile"]["delta_w_limb_asc"].get<double>();
    CE_parameter_storage["limb_desc"] =
        jh.j["contractile"]["limb_desc"].get<double>();
    CE_parameter_storage["limb_asc"] =
        jh.j["contractile"]["limb_asc"].get<double>();
    CE_parameter_storage["a_rel0"] =
        jh.j["contractile"]["a_rel0"].get<double>();
    CE_parameter_storage["b_rel0"] =
        jh.j["contractile"]["b_rel0"].get<double>();
    CE_parameter_storage["s_eccentric"] =
        jh.j["contractile"]["s_eccentric"].get<double>();
    CE_parameter_storage["c_eccentric"] =
        jh.j["contractile"]["c_eccentric"].get<double>();

    // parallel elastic element parameters
    PEE_parameter_storage["L"] = jh.j["parallel_elastic"]["L"].get<double>();
    PEE_parameter_storage["v"] = jh.j["parallel_elastic"]["v"].get<double>();
    PEE_parameter_storage["F"] = jh.j["parallel_elastic"]["F"].get<double>();

    // serial damping element parameters
    SDE_parameter_storage["d_se"] =
        jh.j["serial_damping"]["d_se"].get<double>();
    SDE_parameter_storage["r_se"] =
        jh.j["serial_damping"]["r_se"].get<double>();

    // serial elastic element parameters
    SEE_parameter_storage["l"] = jh.j["serial_elastic"]["l"].get<double>();
    SEE_parameter_storage["delta_u_nll"] =
        jh.j["serial_elastic"]["delta_u_nll"].get<double>();
    SEE_parameter_storage["delta_u_l"] =
        jh.j["serial_elastic"]["delta_u_l"].get<double>();
    SEE_parameter_storage["delta_f"] =
        jh.j["serial_elastic"]["delta_f"].get<double>();

    // length
    length = jh.j["length"].get<double>();
}

void Configuration::load_from_default_json()
{
    std::string path(JSON_DEFAULT_CONFIG_FILE);
    load_from_json(path);
}

}  // namespace hill

}  // namespace pam_models