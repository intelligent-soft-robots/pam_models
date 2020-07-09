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
	MP_l_CE_init_;

	std::function<double(double)> f(std::bind(&HillMuscle::F_sum_init_muscle_force_equilib,
						   this,
						   std::placeholders::_1));
	MP_l_CE_init_ = brents(f, 0, MP_l_MTC_init_, 0.00000001, 1000);
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



