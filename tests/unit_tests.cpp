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
  pam_models::hill::from_default_json();
}

