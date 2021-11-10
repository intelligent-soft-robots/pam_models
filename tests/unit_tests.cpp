#include <unistd.h>
#include <cstdlib>
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

TEST_F(PamModelsHillTests, default_config_factory)
{
    double a = 0.5;
    double l_MTC = 0.05;
    pam_models::hill::from_default_json(a, l_MTC);
}

TEST_F(PamModelsHillTests, compare_to_known)
{
    double ref_force;
    double ref_dot_l_CE;

    // deprecated implementation (considered here ground truth)
    {
        pam_models::hill::deprecated::HillMuscle muscle_test(
            JSON_DEPRECATED_CONFIG_FILE, 0.5, 0.0);
        double l_MTC = 0.05;
        double dot_l_MTC = 0.001;
        double a = 0.5;
        double l_CE = 0.03;
        ref_force =
            muscle_test.get_mucle_tendon_force(l_MTC, dot_l_MTC, a, l_CE);
        ref_dot_l_CE = muscle_test.get_dot_l_CE(l_MTC, dot_l_MTC, a, l_CE);
    }

    // new implementation
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
