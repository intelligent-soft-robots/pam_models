#include "pam_models/hill/factory.hpp"

namespace pam_models
{

  namespace hill
  {

    void from_json(std::string filepath)
    {
      json_helper::Jsonhelper jh;
      try
	{
	  jh.parse(file_path);
	}
      catch(...)
	{
	  std::stringstream ss;
	  ss << "Failed to read JSON file " << file_path << "\n";
	  std::string error = ss.str();
	  throw error;
	}
  }

    
}
