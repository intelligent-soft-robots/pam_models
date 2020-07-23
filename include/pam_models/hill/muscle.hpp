#pragma once

#include "pam_models/hill/contractile_element.hpp"
#include "pam_models/hill/parallel_elastic_element.hpp"
#include "pam_models/hill/serial_damping_element.hpp"
#include "pam_models/hill/serial_elastic_element.hpp"
#include "pam_models/hill/brents.hpp"

namespace pam_models
{

  namespace hill
  {


    double init_muscle_force_equilibrium(double l_CE,
					 const ParallelElasticElement& PEE,
					 const ContractileElement& CE,
					 const SerialElasticElement& SEC,
					 double a_init,
					 double MP_l_MTC_init);


    
    class Muscle
    {

    public:

      Muscle(ContractileElement contractile,
	     ParallelElasticElement parallel_elastic,
	     SerialDampingElement serial_damping,
	     SerialElasticElement serial_elastic,
	     double a_init,
	     double l_MTC_change_init,
	     double length,
	     double f_max);
      std::tuple<double,double> get(double l_MTC,
				    double dot_l_MTC,
				    double a,
				    double l_CE);

    private:


      ContractileElement contractile_;
      ParallelElasticElement parallel_elastic_;
      SerialDampingElement serial_damping_;
      SerialElasticElement serial_elastic_;
      
      /*! initial length of MTC unit, current length = intial length + length from mujoco */
      double MP_l_MTC_init_;
      /*! initial length of CE, current length = intial length + length from mujoco */
      double MP_l_CE_init_;
      /*! initial control signal for muscle tendon unit */
      double a_init_;

    };

  }
  
}



