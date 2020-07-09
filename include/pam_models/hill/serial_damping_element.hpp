#pragma once

#include "pam_models/hill/contractile_element.hpp"

namespace pam_models
{

  namespace hill
  {

    class SerialDampingElement
    {

    public:

      SerialDampingElement(const ContractileElement& contractile_element,
			   double MP_SDE_D_SE,
			   double MP_SDE_R_SE)
	: MP_SDE_D_SE_(MP_SDE_D_SE),
	  MP_SDE_R_SE_(MP_SDE_R_SE),
	  MP_SDE_d_SEmax_( MP_SDE_D_SE *
			   ( contractile_element.MP_CE_F_max* MP_CE_A_rel0 )
			   / ( contractile_element.MP_CE_l_CEopt *
			       contractile_element.MP_CE_B_rel0) )
      {}

      // line 205
      double get_force(double F_CE,
		       double F_PEE,
		       double MP_CE_F_max,
		       double dot_l_MTC,
		       double dot_l_CE)
      {
	double t1 = (1-MP_SDE_R_SE_)*((F_CE+F_PEE)/MP_CE_F_max);
	double t2 = dot_l_MTC-dot_l_CE;
	return MP_SDE_d_SEmax_*(t1+MP_SDE_R_SE_)*(t2);
      }

    private:

      /*! xxx dimensionless factor to scale d_SEmax (Moerl et al., 2012) */
      double MP_SDE_D_SE_;
      /*! minimum value of d_SE normalised to d_SEmax (Moerl et al., 2012) */
      double MP_SDE_R_SE_;
      /*! maximum value in d_SE in [Ns/m] (Moerl et al., 2012) */ 
      double MP_SDE_d_SEmax_;

    };
