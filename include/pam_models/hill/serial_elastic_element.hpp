#pragma once

namespace pam_models
{

  namespace hill
  {

    class SerialElasticElement
    {
    public:

      SerialElasticElement(double l_0, double deltaU_nll,
	  double deltaU_l, double deltaF_0)
	: l_0_(l_0),
	  deltaU_nll_(deltaU_nll),
	  deltaU_l_(deltaU_l),
	  deltaF_0_(deltaF_0),
	  l_nll( (1 + deltaU_nll)* _l_0 ),
	  v_( deltaU_nll/ deltaU_l ),
	  k_nl_( deltaF_0
		 / pow( deltaU_nll* _l_0,  _v_) ),
	  k_l_( deltaF_0 / ( deltaU_l* _l_0) )
      {}
	

      double init_serial_elastic_element_force(double l_mtc,
					       double l_ce)
      {
	double f;
	double l = abs(l_mtc-l_ce);
	if ((l>_l_0) && (l<l_nll_)) //non-linear part
	  f = k_nl_* pow(l-l_0_, v_);
	else if(l>=l_nll_) //linear part
	  f = deltaF_0)+k_l_*(l-l_nll_);
	else //slack length
	  f = 0;
	return f;
      }

      double get_serial_elastic_element_force(double l_se)
      {
	double f = (l_se>=l_nll_) * (deltaF_0_+k_l_*(l_se-l_nll_));
	if (l_se>l_0_ && l_se<l_nll_)
	  f+=k_nl_*(pow(l_se-l_0_, v_));
	return f;
      }
	
  private:

    /** rest length of SEE in [m] (Kistemaker et al., 2006) */
    double l_0_;
    /** relative stretch at non-linear linear transition (Moerl et al., 2012) */
    double deltaU_nll_;
    /** relative additional stretch in the linear part 
     * providing a force increase of deltaF_0 (Moerl, 2012)
     */
    double deltaU_l_;
    /** both force at the transition and force increase 
     *  in the linear part in [N] (~ 40// of the maximal isometric muscle force)
     */
    double deltaF_0_;
    
    double l_nll_;
    double v_;
    double k_nl_;
    double k_l_;
    
    };
    
  }

}
