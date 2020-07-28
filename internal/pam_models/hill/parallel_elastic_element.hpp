#pragma once

#include <math.h>
#include "pam_models/hill/contractile_element.hpp"

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
			     double MP_PEE_F_PEE );
      double get_force(double l_CE);
      
    private:

      friend double init_muscle_force_equilibrium(double,
						  const ParallelElasticElement&,
						  const ContractileElement&,
						  const SerialElasticElement&,
						  double,
						  double);
      friend class Muscle;

      /*! rest length of PEE normalized to optimal lenght of CE (Guenther et al., 2007) */
      double MP_PEE_L_PEE0_;
      /*! exponent of F_PEE (Moerl et al., 2012) */
      double MP_PEE_v_PEE_;
      /*! force of PEE if l_CE is stretched to deltaWlimb_des (Moerl et al., 2012) */
      double MP_PEE_F_PEE_;
      /*! rest length of PEE (Guenther et al., 2007) */
      double MP_PEE_l_PEE0_;
      /*! factor of non-linearity in F_PEE (Guenther et al., 2007) */
      double MP_PEE_K_PEE_;
      
    };

  }

}
