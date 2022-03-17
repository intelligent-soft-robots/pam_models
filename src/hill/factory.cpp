#include "pam_models/hill/factory.hpp"

namespace pam_models
{
namespace hill
{
ContractileElement generate_CE(std::string file_path)
{
    Configuration configuration = generate_configuration_from_path(file_path);

    ContractileElement ce(
        configuration.CE_parameter_storage["f_max"],
        configuration.CE_parameter_storage["l_CEopt"],
        configuration.CE_parameter_storage["delta_w_limb_desc"],
        configuration.CE_parameter_storage["delta_w_limb_asc"],
        configuration.CE_parameter_storage["limb_desc"],
        configuration.CE_parameter_storage["limb_asc"],
        configuration.CE_parameter_storage["a_rel0"],
        configuration.CE_parameter_storage["b_rel0"],
        configuration.CE_parameter_storage["s_eccentric"],
        configuration.CE_parameter_storage["c_eccentric"]);

    return ce;
}

ParallelElasticElement generate_PEE(std::string file_path)
{
    Configuration configuration = generate_configuration_from_path(file_path);
    ContractileElement ce = generate_CE(file_path);

    ParallelElasticElement pee(ce,
                               configuration.PEE_parameter_storage["L"],
                               configuration.PEE_parameter_storage["v"],
                               configuration.PEE_parameter_storage["F"]);
    return pee;
}

SerialDampingElement generate_SDE(std::string file_path)
{
    Configuration configuration = generate_configuration_from_path(file_path);
    ContractileElement ce = generate_CE(file_path);

    SerialDampingElement sde(ce,
                             configuration.SDE_parameter_storage["d_se"],
                             configuration.SDE_parameter_storage["r_se"]);

    return sde;
}

SerialElasticElement generate_SEE(std::string file_path)
{
    Configuration configuration = generate_configuration_from_path(file_path);

    SerialElasticElement see(configuration.SEE_parameter_storage["l"],
                             configuration.SEE_parameter_storage["delta_u_nll"],
                             configuration.SEE_parameter_storage["delta_u_l"],
                             configuration.SEE_parameter_storage["delta_f"]);

    return see;
}

Configuration generate_configuration_from_path(std::string file_path)
{
    Configuration configuration;
    configuration.load_from_json(file_path);

    return configuration;
}

Muscle from_json(std::string file_path, double a_init, double l_MTC_change_init)
{
    Configuration configuration = generate_configuration_from_path(file_path);

    ContractileElement ce = generate_CE(file_path);
    ParallelElasticElement pee = generate_PEE(file_path);
    SerialDampingElement sde = generate_SDE(file_path);
    SerialElasticElement see = generate_SEE(file_path);

    Muscle muscle(
        ce, pee, sde, see, a_init, l_MTC_change_init, configuration.length);

    return muscle;
}

Muscle from_default_json(double a_init, double l_MTC_change_init)
{
    std::string path = Configuration::get_default_json_file().string();
    return from_json(path, a_init, l_MTC_change_init);
}

}  // namespace hill

}  // namespace pam_models
