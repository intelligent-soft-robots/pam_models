#include "pam_models/hill/contractile_element.hpp"

namespace pam_models
{

  namespace hill
  {

    ContractileElement::ContractileElement(double f_max,
					   double MP_CE_l_CEopt,
					   double MP_CE_DeltaW_limb_des,
					   double MP_CE_DeltaW_limb_asc,
					   double MP_CE_v_CElimb_des,
					   double MP_CE_v_CElimb_asc,
					   double MP_CE_A_rel0,
					   double MP_CE_B_rel0,
					   double MP_CE_S_eccentric,
					   double MP_CE_F_eccentric)
      : MP_CE_F_max_(f_max),
	MP_CE_l_CEopt_(MP_CE_l_CEopt),
	MP_CE_DeltaW_limb_des_(MP_CE_DeltaW_limb_des),
	MP_CE_DeltaW_limb_asc_(MP_CE_DeltaW_limb_asc),
	MP_CE_v_CElimb_des_(MP_CE_v_CElimb_des),
	MP_CE_v_CElimb_asc_(MP_CE_v_CElimb_asc),
	MP_CE_A_rel0_(MP_CE_A_rel0),
	MP_CE_B_rel0_(MP_CE_B_rel0),
	MP_CE_S_eccentric_(MP_CE_S_eccentric),
	MP_CE_F_eccentric_(MP_CE_F_eccentric)
    {}
	  
	// line 153
	double ContractileElement::get_isometric_force(double l_CE)
	{
	  if (MP_CE_l_CEopt_<=l_CE)
	    return ( exp( - pow( fabs(  ((l_CE/MP_CE_l_CEopt_)-1)/MP_CE_DeltaW_limb_des_  ),
				 MP_CE_v_CElimb_des_ ) ));
	  if(MP_CE_l_CEopt_>l_CE && MP_CE_l_CEopt_>l_CE)
	    return ( exp( - pow( fabs(  ((l_CE/MP_CE_l_CEopt_)-1)/MP_CE_DeltaW_limb_asc_  ),
				 MP_CE_v_CElimb_asc_ ) ));
	  return 0;

	}

    double ContractileElement::get_a_relative(double l_CE,
					      double F_isom,
					      double a)
    {
      return ( 1.0*(l_CE<MP_CE_l_CEopt_) +
	       F_isom*(l_CE>=MP_CE_l_CEopt_) )*MP_CE_A_rel0_*1/4*(1+3*a);
    }

    double ContractileElement::get_b_relative(double a)
    {
      return MP_CE_B_rel0_*1*1/7*(3+4*a);
    }

  }

}
