#include "gtest/gtest.h"
#include <cstdlib>
#include <unistd.h>
#include <sstream>
#include <set>

#include "pam_models/hill/factory.hpp"


class PamModelsHillTests : public ::testing::Test
{
  void SetUp(){}
  void TearDown(){}
};


TEST_F(PamModelsHillTests,default_config_factory)
{
  double a = 0.5;
  double l_MTC = 0.05;
  pam_models::hill::from_default_json(a,l_MTC);
}


TEST_F(PamModelsHillTests,compare_to_known)
{
  double a_init = 0.5; 
  double l_MTC_init = 0.0;
  pam_models::hill::Muscle muscle = pam_models::hill::from_default_json(a_init,
									l_MTC_init);

  double l_MTC = 0.05;
  double dot_l_MTC = 0.001;
  double a = 0.5;
  double l_CE = 0.03;
  std::tuple<double,double> f_dot_lce = muscle.get(l_MTC,
						   dot_l_MTC,
						   a,
						   l_CE);
  double force = std::get<0>(f_dot_lce);
  double dot_l_CE = std::get<1>(f_dot_lce);

  double ref_force = 2004.1;
  double ref_dot_l_CE = 3.5686;
  
  ASSERT_NEAR(force, ref_force, 0.001);
  ASSERT_NEAR(dot_l_CE, ref_dot_l_CE, 0.001);
}

