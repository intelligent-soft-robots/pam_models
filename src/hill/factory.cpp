#include "pam_models/hill/factory.hpp"

namespace pam_models
{

namespace hill
{

Muscle from_json(std::string file_path, double a_init, double l_MTC_change_init)
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

    double f_max = jh.j["contractile"]["f_max"].get<double>();
    double l_CEopt = jh.j["contractile"]["l_CEopt"].get<double>();
    double delta_w_limb_desc =
        jh.j["contractile"]["delta_w_limb_desc"].get<double>();
    double delta_w_limb_asc =
        jh.j["contractile"]["delta_w_limb_asc"].get<double>();
    double limb_desc = jh.j["contractile"]["limb_desc"].get<double>();
    double limb_asc = jh.j["contractile"]["limb_asc"].get<double>();
    double a_rel0 = jh.j["contractile"]["a_rel0"].get<double>();
    double b_rel0 = jh.j["contractile"]["b_rel0"].get<double>();
    double s_eccentric = jh.j["contractile"]["s_eccentric"].get<double>();
    double c_eccentric = jh.j["contractile"]["c_eccentric"].get<double>();

    ContractileElement ce(f_max,
                          l_CEopt,
                          delta_w_limb_desc,
                          delta_w_limb_asc,
                          limb_desc,
                          limb_asc,
                          a_rel0,
                          b_rel0,
                          s_eccentric,
                          c_eccentric);

    double L = jh.j["parallel_elastic"]["L"].get<double>();
    double v = jh.j["parallel_elastic"]["v"].get<double>();
    double F = jh.j["parallel_elastic"]["F"].get<double>();

    ParallelElasticElement pee(ce, L, v, F);

    double d_se = jh.j["serial_damping"]["d_se"].get<double>();
    double r_se = jh.j["serial_damping"]["r_se"].get<double>();

    SerialDampingElement sde(ce, d_se, r_se);

    double l = jh.j["serial_elastic"]["l"].get<double>();
    double delta_u_nll = jh.j["serial_elastic"]["delta_u_nll"].get<double>();
    double delta_u_l = jh.j["serial_elastic"]["delta_u_l"].get<double>();
    double delta_f = jh.j["serial_elastic"]["delta_f"].get<double>();

    SerialElasticElement see(l, delta_u_nll, delta_u_l, delta_f);

    double length = jh.j["length"].get<double>();

    Muscle muscle(ce, pee, sde, see, a_init, l_MTC_change_init, length);

    return muscle;
}

Muscle from_default_json(double a_init, double l_MTC_change_init)
{
    std::string path(JSON_DEFAULT_CONFIG_FILE);
    return from_json(path, a_init, l_MTC_change_init);
}

}  // namespace hill

}  // namespace pam_models
