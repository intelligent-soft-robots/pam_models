#pragma once

#include "pam_models/hill/brents.hpp"
#include "pam_models/hill/contractile_element.hpp"
#include "pam_models/hill/parallel_elastic_element.hpp"
#include "pam_models/hill/serial_damping_element.hpp"
#include "pam_models/hill/serial_elastic_element.hpp"

namespace pam_models
{
namespace hill
{
/**
 * Calculates muscle force based on inital state of MTC unit.
 *
 * @param l_CE Length of contractile element (CE)
 * @param PEE Parallel elastic element (PEE)
 * @param CE Contractile element (CE)
 * @param SEE Serial elastic element (SEE)
 * @param a_init Initial muscle activation with minimally activated muscle
 * @param MP_l_MTC_init Initial length of the muscle tendon complex (MTC),
 *                      current length is sum of inital length and current
 *                      length received by MuJoCo.
 * @return Muscle force as sum of all individual element forces
 *
 * @remark Line 218
 */
double init_muscle_force_equilibrium(double l_CE,
                                     const ParallelElasticElement& PEE,
                                     const ContractileElement& CE,
                                     const SerialElasticElement& SEC,
                                     double a_init,
                                     double MP_l_MTC_init);

/**
 * @brief Hill-type muscle model
 *
 * Hill-type muscle model with serial damping and eccentric force-velocity
 * relation.
 *
 * @param contractile Contractile element object generated by
 *                    ContractileElement class
 * @param parallel_elastic Parallel elastic object generated by
 *                         ParallelElasticElement class
 * @param serial_damping Serial damping object generated by
 *                       SerialDampingElement class
 * @param serial_elastic Serial elastic object generated by
 *                       SerialElasticElement class
 * @param a_init Initial muscle activation with minimally activated muscle
 * @param l_MTC_change_init Initial length of MTC unit
 * @param length Current muscle length from MuJoCo
 *
 * @cite    D.F.B. Haeufle, M. Günther, A. Bayer, S. Schmitt,
 *          Hill-type muscle model with serial damping and eccentric
 *          force–velocity relation, Journal of Biomechanics,
 *          Volume 47, Issue 6, 2014, Pages 1531-1536,ISSN 0021-9290,
 */
class Muscle
{
public:
    Muscle(ContractileElement contractile,
           ParallelElasticElement parallel_elastic,
           SerialDampingElement serial_damping,
           SerialElasticElement serial_elastic,
           double a_init,
           double l_MTC_change_init,
           double length);

    /**
     * Current force and change of length of the muscle tendon unit are returned
     * based on current length, change of length, muscle activation and length
     * of contractile element.
     *
     * @param l_MTC Current length of MTC unit
     * @param dot_l_MTC Current change of length of MTC unit
     * @param a Muscle activity
     * @param l_CE Current length of contractile element
     * @return Tuple of force and length change of MTC unit.
     */
    std::tuple<double, double> get(double l_MTC,
                                   double dot_l_MTC,
                                   double a,
                                   double l_CE);

private:
    ContractileElement contractile_;
    ParallelElasticElement parallel_elastic_;
    SerialDampingElement serial_damping_;
    SerialElasticElement serial_elastic_;

    /**
     * Initial length of MTC unit, current length = intial length + length from
     * MuJoCo
     */
    double MP_l_MTC_init_;

    /**
     * Initial length of CE, current length = intial length + length from
     * MuJoCo
     */
    double MP_l_CE_init_;

    /**
     * Initial muscle activation / control signal for MTC unit
     */
    double a_init_;
};

}  // namespace hill

}  // namespace pam_models