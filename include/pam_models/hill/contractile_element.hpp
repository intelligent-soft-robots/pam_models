#pragma once

namespace pam_models
{

  namespace hill
  {

    class ContractileElement
    {
    public:
      ContractileElement(double f_max,
			 double MP_CE_l_CEopt,
			 double MP_CE_DeltaW_limb_des,
			 double MP_CE_DeltaW_limb_asc,
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
	  MP_CE_F_eccentric_(MP_CE_F_eccentric),
      {}
      // line 153
      double get_isometric_force(double l_CE)
      {
	if (MP_CE_l_CEopt_<=l_CE)
		return ( exp( - pow( abs(  ((l_CE/MP_CE_l_CEopt_)-1)/MP_CE_DeltaW_limb_des_  ),
				     MP_CE_v_CElimb_des ) ));
	if(MP_CE_l_CEopt>l_CE && MP_CE_l_CEopt>l_CE)
		return ( exp( - pow( abs(  ((l_CE/MP_CE_l_CEopt_)-1)/MP_CE_DeltaW_limb_asc_  ),
				     MP_CE_v_CElimb_asc_ ) ));
	return 0;

      }
    private:
      /*! F_max in [N] for Extensor (Kistemaker et al., 2006) */
      double MP_CE_F_max_;
      /*! optimal length of CE in [m] for Extensor (Kistemaker et al., 2006) */
      double MP_CE_l_CEopt_;
      /*! width of normalized bell curve in descending branch (Moerl et al., 2012) */
      double MP_CE_DeltaW_limb_des_;
      /*! width of normalized bell curve in ascending branch (Moerl et al., 2012) */
      double MP_CE_DeltaW_limb_asc_;
      /*! exponent for descending branch (Moerl et al., 2012) */
      double MP_CE_v_CElimb_des_;
      /*! exponent for ascending branch (Moerl et al., 2012) */
      double MP_CE_v_CElimb_asc_;
      /*! parameter for contraction dynamics: maximum value of A_rel (Guenther, 1997, S. 82) */
      double MP_CE_A_rel0_;
      /*! parameter for contraction dynmacis: maximum value of B_rel (Guenther, 1997, S. 82)*/
      double MP_CE_B_rel0_;
      /*! eccentric force-velocity relation: 
       * relation between F(v) slopes at v_CE=0 (van Soest & Bobbert, 1993)*/
      double MP_CE_S_eccentric_;
      /*! eccentric force-velocity relation: 
       *  factor by which the force can exceed F_isom for large eccentric velocities
       * (van Soest & Bobbert, 1993) */
      double MP_CE_F_eccentric_;

    };
    

  }

}
