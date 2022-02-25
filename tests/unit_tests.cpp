#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>

#include "gtest/gtest.h"
#include "pam_models/hill/deprecated/deprecated_muscle.hpp"
#include "pam_models/hill/factory.hpp"
class PamModelsHillTests : public ::testing::Test
{
    void SetUp()
    {
    }
    void TearDown()
    {
    }
};

/**
 * Compares found roots with Brent's method of lambda test function
 * with known roots of test function.
 */
TEST_F(PamModelsHillTests, check_brents_method)
{
    // General test parameters
    // lambda test function with known roots
    auto poly = [](double x) { return (x + 3) * (x - 1) * (x - 1); };

    std::function<double(double)> f = poly;
    double lower = -4.0;
    double upper = 0.75;
    double tol = 0.001;
    unsigned int max_iter = 1000U;

    // Test root search
    double root = pam_models::hill::brents(f, lower, upper, tol, max_iter);
    double root_solution = -3.0;

    ASSERT_NEAR(root, root_solution, 0.0001);
}

/**
 * Unit test generates json file with test parameter and compares loaded
 * parameter from json loader with specified test parameter.
 */
TEST_F(PamModelsHillTests, check_json_loader)
{
    char path[] = "/tmp/test_json_XXXXXX";
    // generates unique and secure file for loading test parameter
    int unique_file_specifier = mkstemp(path);

    if (unique_file_specifier == -1)
    {
        throw std::runtime_error("Failed to generate test file");
    }

    std::string file_path = path;

    // Randomly generate muscle parameters and write into test file
    pam_models::hill::Configuration configuration;

    json j;

    j["length"] = (double)std::rand() / RAND_MAX;

    j["contractile"]["f_max"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["l_CEopt"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["delta_w_limb_desc"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["delta_w_limb_asc"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["limb_desc"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["limb_asc"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["a_rel0"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["b_rel0"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["s_eccentric"] = (double)std::rand() / RAND_MAX;
    j["contractile"]["c_eccentric"] = (double)std::rand() / RAND_MAX;

    j["serial_damping"]["d_se"] = (double)std::rand() / RAND_MAX;
    j["serial_damping"]["r_se"] = (double)std::rand() / RAND_MAX;

    j["parallel_elastic"]["L"] = (double)std::rand() / RAND_MAX;
    j["parallel_elastic"]["v"] = (double)std::rand() / RAND_MAX;
    j["parallel_elastic"]["F"] = (double)std::rand() / RAND_MAX;

    j["serial_elastic"]["l"] = (double)std::rand() / RAND_MAX;
    j["serial_elastic"]["delta_u_nll"] = (double)std::rand() / RAND_MAX;
    j["serial_elastic"]["delta_u_l"] = (double)std::rand() / RAND_MAX;
    j["serial_elastic"]["delta_f"] = (double)std::rand() / RAND_MAX;

    std::ofstream o(file_path);
    o << std::setw(4) << j << std::endl;

    // Compares generated and loaded parameters from test file
    configuration.load_from_json(file_path);

    ASSERT_NEAR(j["length"], configuration.length, 0.001);

    ASSERT_NEAR(j["contractile"]["f_max"],
                configuration.CE_parameter_storage["f_max"],
                0.001);
    ASSERT_NEAR(j["contractile"]["l_CEopt"],
                configuration.CE_parameter_storage["l_CEopt"],
                0.001);
    ASSERT_NEAR(j["contractile"]["delta_w_limb_desc"],
                configuration.CE_parameter_storage["delta_w_limb_desc"],
                0.001);
    ASSERT_NEAR(j["contractile"]["delta_w_limb_asc"],
                configuration.CE_parameter_storage["delta_w_limb_asc"],
                0.001);
    ASSERT_NEAR(j["contractile"]["limb_desc"],
                configuration.CE_parameter_storage["limb_desc"],
                0.001);
    ASSERT_NEAR(j["contractile"]["limb_asc"],
                configuration.CE_parameter_storage["limb_asc"],
                0.001);
    ASSERT_NEAR(j["contractile"]["a_rel0"],
                configuration.CE_parameter_storage["a_rel0"],
                0.001);
    ASSERT_NEAR(j["contractile"]["b_rel0"],
                configuration.CE_parameter_storage["b_rel0"],
                0.001);
    ASSERT_NEAR(j["contractile"]["s_eccentric"],
                configuration.CE_parameter_storage["s_eccentric"],
                0.001);
    ASSERT_NEAR(j["contractile"]["c_eccentric"],
                configuration.CE_parameter_storage["c_eccentric"],
                0.001);

    ASSERT_NEAR(j["parallel_elastic"]["L"],
                configuration.PEE_parameter_storage["L"],
                0.001);
    ASSERT_NEAR(j["parallel_elastic"]["v"],
                configuration.PEE_parameter_storage["v"],
                0.001);
    ASSERT_NEAR(j["parallel_elastic"]["F"],
                configuration.PEE_parameter_storage["F"],
                0.001);

    ASSERT_NEAR(j["serial_damping"]["d_se"],
                configuration.SDE_parameter_storage["d_se"],
                0.001);
    ASSERT_NEAR(j["serial_damping"]["r_se"],
                configuration.SDE_parameter_storage["r_se"],
                0.001);

    ASSERT_NEAR(j["serial_elastic"]["l"],
                configuration.SEE_parameter_storage["l"],
                0.001);
    ASSERT_NEAR(j["serial_elastic"]["delta_u_nll"],
                configuration.SEE_parameter_storage["delta_u_nll"],
                0.001);
    ASSERT_NEAR(j["serial_elastic"]["delta_u_l"],
                configuration.SEE_parameter_storage["delta_u_l"],
                0.001);
    ASSERT_NEAR(j["serial_elastic"]["delta_f"],
                configuration.SEE_parameter_storage["delta_f"],
                0.001);
}

/**
 * Comparing resulting muscle forces of this package to depricated
 * implementation of Hill-type muscle model.
 */
TEST_F(PamModelsHillTests, compare_to_depricated_muscle_model)
{
    double ref_force;
    double ref_dot_l_CE;

    // Deprecated implementation (considered here ground truth)
    {
        double a_init = 0.5;
        double l_MTC_init = 0.0;

        pam_models::hill::deprecated::HillMuscle muscle_test(
            JSON_DEPRECATED_CONFIG_FILE, a_init, l_MTC_init);

        double l_MTC = 0.05;
        double dot_l_MTC = 0.001;
        double a = 0.5;
        double l_CE = 0.03;

        ref_force =
            muscle_test.get_mucle_tendon_force(l_MTC, dot_l_MTC, a, l_CE);
        ref_dot_l_CE = muscle_test.get_dot_l_CE(l_MTC, dot_l_MTC, a, l_CE);
    }

    // New implementation
    {
        double a_init = 0.5;
        double l_MTC_init = 0.0;

        pam_models::hill::Muscle muscle = pam_models::hill::from_json(
            JSON_TEST_CONFIG_FILE, a_init, l_MTC_init);

        double l_MTC = 0.05;
        double dot_l_MTC = 0.001;
        double a = 0.5;
        double l_CE = 0.03;

        std::tuple<double, double> f_dot_lce =
            muscle.get(l_MTC, dot_l_MTC, a, l_CE);

        double force = std::get<0>(f_dot_lce);
        double dot_l_CE = std::get<1>(f_dot_lce);

        ASSERT_NEAR(force, ref_force, 0.001);
        ASSERT_NEAR(dot_l_CE, ref_dot_l_CE, 0.001);
    }
}

/**
 * Testing implementation of the calculation of the isometric force of the
 * contractile element to reference calculation of the isometric force
 * equations.
 */
TEST_F(PamModelsHillTests, CE_compare_isometric_force)
{
    // General test parameters
    double l_CE = 0.25;

    // Model parameters
    std::string file_path = JSON_DEFAULT_CONFIG_FILE;
    pam_models::hill::Configuration config;
    config.load_from_json(file_path);

    double MP_CE_l_CEopt = config.CE_parameter_storage["l_CEopt"];
    double MP_CE_DeltaW_limb_des =
        config.CE_parameter_storage["delta_w_limb_desc"];
    double MP_CE_v_CElimb_des = config.CE_parameter_storage["delta_w_limb_asc"];
    double MP_CE_DeltaW_limb_asc = config.CE_parameter_storage["limb_desc"];
    double MP_CE_v_CElimb_asc = config.CE_parameter_storage["limb_asc"];

    pam_models::hill::ContractileElement ce =
        pam_models::hill::generate_CE(file_path);

    // Test isometric force of contractile element
    double F_isom = ce.get_isometric_force(l_CE);
    double F_isom_ref;

    if (MP_CE_l_CEopt <= l_CE)
    {
        F_isom_ref = (exp(
            -pow(fabs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_des),
                 MP_CE_v_CElimb_des)));
    }
    else if (MP_CE_l_CEopt > l_CE)
    {
        F_isom_ref = (exp(
            -pow(fabs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_asc),
                 MP_CE_v_CElimb_asc)));
    }
    else
    {
        F_isom_ref = 0.0;
    }

    ASSERT_NEAR(F_isom_ref, F_isom, 0.001);
}

/**
 * Testing implementation of contractile element by comparing internal
 * calculations to reference implementation of SEE equations. Tested is
 * the calculation of the model parameters A_rel, B_rel as well as
 * the resulting CE force.
 */
TEST_F(PamModelsHillTests, CE_compare_relative_curves)
{
    // General parameters
    double l_CE = 0.25;
    double a = 0.05;

    // Model parameters
    std::string file_path = JSON_DEFAULT_CONFIG_FILE;
    pam_models::hill::Configuration config;
    config.load_from_json(file_path);

    double MP_CE_l_CEopt = config.CE_parameter_storage["l_CEopt"];
    double MP_CE_A_rel0 = config.CE_parameter_storage["a_rel0"];
    double MP_CE_B_rel0 = config.CE_parameter_storage["b_rel0"];

    pam_models::hill::ContractileElement ce =
        pam_models::hill::generate_CE(file_path);

    // Test relative a
    double F_isom = ce.get_isometric_force(l_CE);
    double A_rel = ce.get_a_relative(l_CE, F_isom, a);

    double L_A_rel;
    double Q_A_rel;
    double A_rel_ref;

    if (l_CE < MP_CE_l_CEopt)
    {
        L_A_rel = 1.0;
    }
    else
    {
        L_A_rel = F_isom;
    }

    Q_A_rel = 1.0 / 4.0 * (1.0 + 3.0 * a);
    A_rel_ref = MP_CE_A_rel0 * L_A_rel * Q_A_rel;

    ASSERT_NEAR(A_rel, A_rel_ref, 0.001);

    // Test relative b
    double B_rel = ce.get_b_relative(a);

    double L_B_rel;
    double Q_B_rel;
    double B_rel_ref;

    L_B_rel = 1.0;
    Q_B_rel = (1.0 / 7.0) * (3 + 4 * a);
    B_rel_ref = MP_CE_B_rel0 * L_B_rel * Q_B_rel;

    ASSERT_NEAR(B_rel, B_rel_ref, 0.001);
}

/**
 * Testing implementation of parallel elastic element by comparing
 * internal calculations to reference implementation of PEE
 * equations. Tested is the calculation of the model parameters
 * l_PEE0, K_PEE as well as the resulting PEE force.
 */
TEST_F(PamModelsHillTests, PEE_compare_to_reference)
{
    // General test parameters
    double l_CE = 0.25;

    // Model parameters
    std::string file_path = JSON_DEFAULT_CONFIG_FILE;
    pam_models::hill::Configuration config;
    config.load_from_json(file_path);

    double MP_PEE_L_PEE0 = config.PEE_parameter_storage["L"];
    double MP_PEE_v_PEE = config.PEE_parameter_storage["v"];
    double MP_PEE_F_PEE = config.PEE_parameter_storage["F"];

    double MP_CE_l_CEopt = config.CE_parameter_storage["l_CEopt"];
    double MP_CE_F_max = config.CE_parameter_storage["f_max"];
    double MP_CE_DeltaW_limb_desc =
        config.CE_parameter_storage["delta_w_limb_desc"];

    pam_models::hill::ParallelElasticElement pee =
        pam_models::hill::generate_PEE(file_path);

    // Test free parameter l_PEE
    double MP_PEE_l_PEE0 = pee.get_l_parameter();
    double MP_PEE_l_PEE0_ref = MP_PEE_L_PEE0 * MP_CE_l_CEopt;

    ASSERT_NEAR(MP_PEE_l_PEE0, MP_PEE_l_PEE0_ref, 0.001);

    // Test K_PEE
    double MP_PEE_K_PEE = pee.get_K_parameter();
    double MP_PEE_K_PEE_ref =
        MP_PEE_F_PEE *
        (MP_CE_F_max /
         pow(MP_CE_l_CEopt * (MP_CE_DeltaW_limb_desc + 1 - MP_PEE_L_PEE0),
             MP_PEE_v_PEE));

    ASSERT_NEAR(MP_PEE_K_PEE, MP_PEE_K_PEE_ref, 0.001);

    // Test F_PEE
    double F_PEE = pee.get_force(l_CE);
    double F_PEE_ref;

    if (l_CE >= MP_PEE_l_PEE0)
    {
        F_PEE_ref = MP_PEE_K_PEE * pow(l_CE - MP_PEE_l_PEE0, MP_PEE_v_PEE);
    }

    if (l_CE < MP_PEE_l_PEE0)
    {
        F_PEE_ref = 0;
    }

    ASSERT_NEAR(F_PEE, F_PEE_ref, 0.001);
}

/**
 * Testing implementation of serial damping element by comparing internal
 * calculations to reference implementation of SDE equations. Tested is
 * the calculation of the maximum damping coefficient and the SDE force.
 */
TEST_F(PamModelsHillTests, SDE_compare_to_reference)
{
    // General test parameters
    double F_CE = 0.5;
    double F_PEE = 0.2;
    double dot_l_MTC = 0.5;
    double dot_l_CE = 0.5;

    // Model parameters
    std::string file_path = JSON_DEFAULT_CONFIG_FILE;
    pam_models::hill::Configuration config;
    config.load_from_json(file_path);

    double MP_CE_F_max = config.CE_parameter_storage["f_max"];
    double MP_SDE_R_SE = config.SDE_parameter_storage["r_se"];
    double MP_SDE_D_SE = config.SDE_parameter_storage["d_se"];

    double MP_CE_A_rel0 = config.CE_parameter_storage["a_rel0"];
    double MP_CE_l_CEopt = config.CE_parameter_storage["l_CEopt"];
    double MP_CE_B_rel0 = config.CE_parameter_storage["b_rel0"];

    pam_models::hill::SerialDampingElement sde =
        pam_models::hill::generate_SDE(file_path);

    // Test maximum damping coefficient
    double MP_SDE_d_SEmax = sde.get_maximum_damping_coefficent();
    double MP_SDE_d_SEmax_ref = MP_SDE_D_SE * (MP_CE_F_max * MP_CE_A_rel0) /
                                (MP_CE_l_CEopt * MP_CE_B_rel0);

    ASSERT_NEAR(MP_SDE_d_SEmax_ref, MP_SDE_d_SEmax, 0.001);

    // Test SDE force generation
    double F_SDE = sde.get_force(F_CE, F_PEE, MP_CE_F_max, dot_l_MTC, dot_l_CE);
    double F_SDE_ref;

    double t1 = (1 - MP_SDE_R_SE) * ((F_CE + F_PEE) / MP_CE_F_max);
    double t2 = dot_l_MTC - dot_l_CE;

    F_SDE_ref = MP_SDE_d_SEmax_ref * (t1 + MP_SDE_R_SE) * (t2);

    ASSERT_NEAR(F_SDE_ref, F_SDE, 0.001);
}

/**
 * Testing implementation of serial elastic element by comparing internal
 * calculations to reference implementation of SEE equations. Tested is
 * the calculation of the free model parameters l_nll, v, k_l and k_nl
 * as well as the resulting SEE force.
 */
TEST_F(PamModelsHillTests, SEE_compare_to_reference)
{
    // General test parameters
    double l_MTC = 0.35;
    double l_CE = 0.1;

    // Model parameters
    std::string file_path = JSON_DEFAULT_CONFIG_FILE;
    pam_models::hill::Configuration config;
    config.load_from_json(file_path);

    double l_0 = config.SEE_parameter_storage["l"];
    double deltaU_nll = config.SEE_parameter_storage["delta_u_nll"];
    double deltaU_l = config.SEE_parameter_storage["delta_u_l"];
    double deltaF_0 = config.SEE_parameter_storage["delta_f"];

    pam_models::hill::SerialElasticElement see =
        pam_models::hill::generate_SEE(file_path);

    // Overall parameter tests
    double l_nll = see.get_l_nll();
    double v = see.get_v();
    double k_l = see.get_k_l();
    double k_nl = see.get_k_nl();

    double l_nll_ref = l_0 * (1 + deltaU_nll);
    double v_ref = deltaU_nll / deltaU_l;
    double k_nl_ref = deltaF_0 / pow(deltaU_nll * l_0, v_ref);
    double k_l_ref = deltaF_0 / (deltaU_l * l_0);

    ASSERT_NEAR(l_nll_ref, l_nll, 0.001);
    ASSERT_NEAR(v_ref, v, 0.001);
    ASSERT_NEAR(k_l_ref, k_l, 0.001);
    ASSERT_NEAR(k_nl_ref, k_nl, 0.001);

    ASSERT_TRUE(l_0 <= l_nll_ref);

    // F_SEE test
    double F_SEE = see.get_force(l_MTC, l_CE);
    double F_SEE_ref;

    double l_SEE = abs(l_MTC - l_CE);

    if (l_SEE < l_0)
    {
        F_SEE_ref = 0.0;
    }

    if ((l_SEE < l_nll_ref) && (l_SEE > l_0))
    {
        F_SEE_ref = k_nl_ref * pow(l_SEE - l_0, v_ref);
    }

    if (l_SEE >= l_nll_ref)
    {
        F_SEE_ref = deltaF_0 + k_l_ref * (l_SEE - l_nll_ref);
    }

    ASSERT_NEAR(F_SEE_ref, F_SEE, 0.001);
}