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

