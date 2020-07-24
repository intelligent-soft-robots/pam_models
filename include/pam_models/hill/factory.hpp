#pragma once

#include <iostream>
#include <string>
#include "json_helper/json_helper.hpp"
#include "pam_models/hill/muscle.hpp"

namespace pam_models
{


  namespace hill
  {

    void from_json(std::string file_path);
    void from_default_json();
    
  }

}
