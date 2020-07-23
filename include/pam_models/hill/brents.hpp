#pragma once

#include <functional>
#include <stdexcept>

namespace pam_models
{

  namespace hill
  {

    double brents(std::function<double (double)> f,
		  double lower,
		  double upper,
		  double tol=0.00000001,
		  unsigned int max_iter=1000);

  }
  
}
