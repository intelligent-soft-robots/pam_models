#pragma once

#include <iostream>
#include <string>
#include "json_helper/json_helper.hpp"
#include "pam_models/hill/muscle.hpp"

namespace pam_models
{


  namespace hill
  {

    Muscle from_json(std::string file_path,
		   double a_init,
		   double l_MTC_change_init);
    Muscle from_default_json(double a_init,
			     double l_MTC_change_init);
  }

}
