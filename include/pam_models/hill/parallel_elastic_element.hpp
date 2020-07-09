#pragma once

#include "pam_models/contractile_elements.hpp"

namespace pam_models
{

  namespace hill
  {

    class ParallelElasticElement
    {

    public:

      ParallelElasticElement(const ContractileElement& contractile_element,
			     double MP_PEE_L_PEE0,
			     double MP_PEE_v_PEE,
			     double MP_PEE_F_PEE )
	: MP_PEE_L_PEE0_(MP_PEE_L_PEE0),
	  MP_PEE_v_PEE_(MP_PEE_v_PEE),
	  MP_PEE_F_PEE_(MP_PEE_F_PEE),
	  MP_PEE_l_PEE0_(MP_PEE_L_PEE0* contractile_element.MP_CE_l_CEopt),
	  MP_PEE_K_PEE ( MP_PEE_F_PEE *
			 ( MP_CE_F_max /
			   pow( contractile_element.MP_CE_l_CEopt *
				( contractile_element.MP_CE_DeltaW_limb_des+1-MP_PEE_L_PEE0),
				MP_PEE_v_PEE )) )
      {}
    private:

      /*! rest length of PEE normalized to optimal lenght of CE (Guenther et al., 2007) */
      double MP_PEE_L_PEE0;
      /*! exponent of F_PEE (Moerl et al., 2012) */
      double MP_PEE_v_PEE;
      /*! force of PEE if l_CE is stretched to deltaWlimb_des (Moerl et al., 2012) */
      double MP_PEE_F_PEE;
      /*! rest length of PEE (Guenther et al., 2007) */
      double MP_PEE_l_PEE0;
      /*! factor of non-linearity in F_PEE (Guenther et al., 2007) */
      double MP_PEE_K_PEE;
      
    };

  }

}
