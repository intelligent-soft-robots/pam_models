#include "pam_models/hill/parallel_elastic_element.hpp"

namespace pam_models
{

  namespace hill
  {

    ParallelElasticElement::ParallelElasticElement(const ContractileElement& contractile_element,
			     double MP_PEE_L_PEE0,
			     double MP_PEE_v_PEE,
			     double MP_PEE_F_PEE )
	: MP_PEE_L_PEE0_(MP_PEE_L_PEE0),
	  MP_PEE_v_PEE_(MP_PEE_v_PEE),
	  MP_PEE_F_PEE_(MP_PEE_F_PEE),
	  MP_PEE_l_PEE0_(MP_PEE_L_PEE0* contractile_element.MP_CE_l_CEopt_),
	  MP_PEE_K_PEE_ ( MP_PEE_F_PEE_ *
			 ( contractile_element.MP_CE_F_max_ /
			   pow( contractile_element.MP_CE_l_CEopt_ *
				( contractile_element.MP_CE_DeltaW_limb_des_+1-MP_PEE_L_PEE0),
				MP_PEE_v_PEE )) )
      {}

    double ParallelElasticElement::get_force(double l_CE)
      {
	if(l_CE>=MP_PEE_l_PEE0_)
	  {
	    return MP_PEE_K_PEE_*pow(l_CE-MP_PEE_l_PEE0_, MP_PEE_v_PEE_);
	  }
	return 0;
      }
      

  }

}
