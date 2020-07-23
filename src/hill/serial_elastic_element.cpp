#include "pam_models/hill/serial_elastic_element.hpp"

namespace pam_models
{

  namespace hill
  {

    SerialElasticElement::SerialElasticElement(double l_0, double deltaU_nll,
					       double deltaU_l, double deltaF_0)
      : l_0_(l_0),
	deltaU_nll_(deltaU_nll),
	deltaU_l_(deltaU_l),
	deltaF_0_(deltaF_0),
	l_nll_( (1 + deltaU_nll)* l_0 ),
	v_( deltaU_nll/ deltaU_l ),
	k_nl_( deltaF_0
	       / pow( deltaU_nll* l_0,  v_) ),
	k_l_( deltaF_0 / ( deltaU_l* l_0) )
    {}
	

    double SerialElasticElement::init_serial_elastic_element_force(double l_mtc,
								   double l_ce)
    {
      double f;
      double l = fabs(l_mtc-l_ce);
      //non-linear part
      if ((l>l_0_) && (l<l_nll_))
	{
	  f = k_nl_* pow(l-l_0_, v_);
	}
      //linear part
      else if(l>=l_nll_)
	{
	  f = deltaF_0_+k_l_*(l-l_nll_);
	}
      //slack length
      else
	{
	  f = 0;
	}
      return f;
  }

  double SerialElasticElement::get_force(double l_MTC,double l_CE)
  {
    double l_se = l_MTC-l_CE;
    double f = (l_se>=l_nll_) * (deltaF_0_+k_l_*(l_se-l_nll_));
    if (l_se>l_0_ && l_se<l_nll_)
      f+=k_nl_*(pow(l_se-l_0_, v_));
    return f;
  }
	
}
  
}

