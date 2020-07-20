#pragma once

#include "pam_models/hill/contractile_element.hpp"
#include "pam_models/hill/parallel_elastic_element.hpp"
#include "pam_models/hill/serial_damping_element.hpp"
#include "pam_models/hill/serial_elastic_element.hpp"
#include "pam_models/hill/brent.hpp"

namespace pam_models
{

  namespace hill
  {


    // line 218
    double init_muscle_force_equilibrium(double l_CE,
					 const ParallelElasticElement& PEE,
					 const ContractilElement& CE,
					 const SerialElasticElement& SEC,
					 double a_init,
					 double MP_l_MTC_init,
					 
    {
      double l_MTC = MP_l_MTC_init;

      // Isometric force (Force length relation)
      //Guenther et al. 2007
      double f_isom;
      if(l_CE >= CE.MP_CE_l_CEopt_) //descending branch
	{
	  F_isom =
	    exp( - pow( abs( ((l_CE/CE.MP_CE_l_CEopt_)-1)/CE.MP_CE_DeltaW_limb_des_ ) ,
			CE.MP_CE_v_CElimb_des_) );
	}
      else //ascending branch
	{
        F_isom =
	  exp( - pow( abs( ((l_CE/CE.MP_CE_l_CEopt_)-1)/CE.MP_CE_DeltaW_limb_asc_ ) ,
		      CE.MP_CE_v_CElimb_asc_ ) );
	}
      
      // Force of the parallel elastic element
      double F_PEE;
      if(l_CE >= PEE.MP_PEE_l_PEE0_)
	{
	  F_PEE = PEE.MP_PEE_K_PEE_ * pow(l_CE-PEE.MP_PEE_l_PEE0_, PEE.MP_PEE_v_PEE_);
	}
      else // shorter than slack length
	{
	  F_PEE = 0;
	}

      // Force of the serial elastic element
      double l_SEE = abs(l_MTC-l_CE);
      if ((l_SEE>SEE.l_0_) && (l_SEE<SEE.l_nll_)) //non-linear part
	{
	  F_SEE = SEE.k_nl_* pow(l_SEE-SEE.l_0_, SEE.v_);
	}
      else if(l_SEE>=SEE.l_nll_) //linear part
	{
	  F_SEE = SEE..deltaF_0_+SEE.k_l_*(l_SEE-SEE.l_nll_);
	}
      else //slack length
	{
	  F_SEE = 0;
	}

      // Contractile element force (isometric)
      double F_CE = CE.MP_CE_F_max_*a_init*F_isom;

      double F_sum = F_SEE-F_CE-F_PEE;

      return F_sum;
    }


    
    class Muscle
    {

    public:

      Muscle(double a_init,
	     double l_MTC_change_init,
	     double length,
	     double f_max)
	: MP_l_MTC_init_(length+l_MTC_change_init),
	  a_init_(a_init)
      {
	std::function<double(double)> f(std::bind(init_muscle_force_equilibrium,
						  std::placeholders::_1,
						  parallel_elastic_,
						  contractile_,
						  serial_elastic_,
						  a_init,
						  MP_l_MTC_init_));
	MP_l_CE_init_ = brents(f, 0, MP_l_MTC_init_);
      }


      // line 142, returns F_MTU_current and dot_l_CE_current
      std::tuple<double,double> Muscle::get(double l_MTC, double dot_l_MTC, double a, double l_CE)
      {
	
	l_MTC = l_MTC + MP_l_MTC_init;
	l_CE = l_CE + MP_l_CE_init;

	double F_isom = contractile_.get_isometric_force(l_CE);
	double F_PEE = parallel_elastic_.get_force(l_CE);
	double F_SEE = serial_elastic.get_force(l_MTC,l_CE);
	double A_rel = contractile.get_a_relative(l_CE,F_isom);
	double B_rel = contractile.get_b_relative();

	double D_0;
	{
	  D_0 = contractile_.MP_CE_l_CEopt_ * B_rel * serial_damping_.MP_SDE_d_SEmax;
	  D_0 *= ( serial_damping.MP_SDE_R_SE + (1-serial_damping_.MP_SDE_R_SE) *
		   ( a * F_isom + F_PEE/contractile_.MP_CE_F_max ) );
	}

	double C_2;
	{
	  double rel = A_rel - F_PEE / contractile_.MP_CE_F_max_;
	  double one_min_ = 1.0 - serial_damping_.MP_SDE_R_SE_;
	  C_2= MP_SDE_d_SEmax * ( serial_damping_.MP_SDE_R_SE_ - rel  * one_min );
	}

	double C_1 = - ( C_2*dot_l_MTC + D_0 + F_SEE - F_PEE + (contractile_.MP_CE_F_max_*A_rel) );

	double C_0;
	{
	  double f_s = F_SEE - F_PEE - contractile_.MP_CE_F_max_*a*F_isom
	  C_0= D_0*dot_l_MTC +
	    contractile_.MP_CE_l_CEopt * B_rel * f_s;
	}
	
	//quadratic equation for concentric contractions (-sqrt)
	double dot_l_CE = (-C_1-sqrt(C_1*C_1-4*C_2*C_0))/(2*C_2);

	if(dot_l_CE>0)  //dot_l_CE > 0 -> eccentric contraction -> recalculate dot_l_CE
	  {
	    double A_rel_con = A_rel;
	    double B_rel_con = B_rel;
	    A_rel = -MP_CE_F_eccentric*a*F_isom;
	    B_rel = a*F_isom*(1-MP_CE_F_eccentric)/(a*F_isom+A_rel_con)*B_rel_con/MP_CE_S_eccentric;
	    D_0 = MP_CE_l_CEopt * B_rel * MP_SDE_d_SEmax * ( MP_SDE_R_SE + (1-MP_SDE_R_SE) * ( a * F_isom + F_PEE/MP_CE_F_max ) );
	    C_2 = MP_SDE_d_SEmax * ( MP_SDE_R_SE - ( A_rel - F_PEE/MP_CE_F_max ) * (1 - MP_SDE_R_SE) );
	    C_1 = - ( C_2*dot_l_MTC + D_0 + F_SEE - F_PEE + (MP_CE_F_max*A_rel) );
	    C_0 = D_0*dot_l_MTC + MP_CE_l_CEopt * B_rel * ( F_SEE - F_PEE - MP_CE_F_max*a*F_isom );
	    dot_l_CE = (-C_1+sqrt(C_1*C_1-4*C_2*C_0))/(2*C_2); //quadratic equation for eccentric contractions (+sqrt)
	    //printf("hillMuscle-dot_l_CE: %f (recalculated)\n", dot_l_CE);
	  }

	dot_l_CE_current = dot_l_CE;

	double F_CE = MP_CE_F_max * (  ( (a*F_isom+A_rel) / ( 1- (dot_l_CE/(B_rel*MP_CE_l_CEopt)) ) )-A_rel );

	double F_SDE = MP_SDE_d_SEmax*((1-MP_SDE_R_SE)*((F_CE+F_PEE)/MP_CE_F_max)+MP_SDE_R_SE)*(dot_l_MTC-dot_l_CE);

	F_MTU_current = F_SEE + F_SDE;

      }
      
      
    private:


      ContractileElement contractile_;
      ParallelElasticElement parallel_elastic_;
      SerialDampingElement serial_damping_;
      SerialElasticElement serial_elastic_;
      
      /*! initial length of MTC unit, current length = intial length + length from mujoco */
      double MP_l_MTC_init;
      /*! initial length of CE, current length = intial length + length from mujoco */
      double MP_l_CE_init;
      /*! initial control signal for muscle tendon unit */
      double a_init;

      bool F_MTU_current_and_dot_l_CE_current_exist = false;
      double F_MTU_current;
      double dot_l_CE_current;

      double l_MTC_last_recalc;
      double dot_l_MTC_last_recalc;
      double a_last_recalc; 
      double l_CE_last_recalc;

      
    };

  }
  
}



