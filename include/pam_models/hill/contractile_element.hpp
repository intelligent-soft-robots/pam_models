#pragma once

#include <math.h>

namespace pam_models
{
namespace hill
{
// forward declaration for
// using friend keyword
class ParallelElasticElement;
class SerialDampingElement;
class Muscle;
class ContractileElement;
class SerialElasticElement;
double init_muscle_force_equilibrium(double,
                                     const ParallelElasticElement&,
                                     const ContractileElement&,
                                     const SerialElasticElement&,
                                     double,
                                     double);

/**
 * @brief Contractile element (CE)
 *
 * Contractile element (CE) of the muscle tendon complex (MTC) model.
 *
 * @details The contracticle element models the active force production
 *          by incorporating force-length and force-velocity
 *          dependencies of the model of the muscle tendon complex.
 */
class ContractileElement
{
public:
    ContractileElement(double f_max,
                       double MP_CE_l_CEopt,
                       double MP_CE_DeltaW_limb_des,
                       double MP_CE_DeltaW_limb_asc,
                       double MP_CE_v_CElimb_des,
                       double MP_CE_v_CElimb_asc,
                       double MP_CE_A_rel0,
                       double MP_CE_B_rel0,
                       double MP_CE_S_eccentric,
                       double MP_CE_F_eccentric);

    /**
     * Gets isometric force of CE.
     *
     * @param l_CE Length of CE
     * @return Isometric force
     */
    double get_isometric_force(double l_CE);

    /**
     * Gets the normalized Hill-parameter a (Hill, 1938).
     *
     * @param l_CE Length of CE
     * @param F_isom Isometric force
     * @param a Muscle activation
     * @return Value of Hill parameter a
     */
    double get_a_relative(double l_CE, double F_isom, double a);

    /**
     * Gets the normalied Hill-parameter b (Hill, 1938).
     *
     * @param a Muscle activation
     * @return Value of Hill parameter b
     */
    double get_b_relative(double a);

private:
    /**
     * Calculates muscle force based on inital state of MTC unit.
     *
     * @return Muscle force as sum of all individual element forces
     */
    friend double init_muscle_force_equilibrium(double,
                                                const ParallelElasticElement&,
                                                const ContractileElement&,
                                                const SerialElasticElement&,
                                                double,
                                                double);
    friend class ParallelElasticElement;
    friend class SerialDampingElement;
    friend class Muscle;

    /**
     * F_max in [N] for Extensor (Kistemaker et al., 2006)
     */
    double MP_CE_F_max_;

    /**
     * Optimal length of CE in [m] for Extensor (Kistemaker et al., 2006)
     */
    double MP_CE_l_CEopt_;

    /**
     * Width of normalized bell curve in descending branch (Moerl et al., 2012)
     */
    double MP_CE_DeltaW_limb_des_;

    /**
     * Width of normalized bell curve in ascending branch (Moerl et al., 2012)
     */
    double MP_CE_DeltaW_limb_asc_;

    /**
     * Exponent for descending branch (Moerl et al., 2012)
     */
    double MP_CE_v_CElimb_des_;

    /**
     * Exponent for ascending branch (Moerl et al., 2012)
     */
    double MP_CE_v_CElimb_asc_;

    /**
     * Parameter for contraction dynamics: Maximum value of A_rel (Guenther,
     * 1997, S. 82)
     */
    double MP_CE_A_rel0_;

    /**
     * Parameter for contraction dynamics: Maximum value of B_rel (Guenther,
     *  1997, S. 82)
     */
    double MP_CE_B_rel0_;

    /**
     * Eccentric force-velocity relation: Relation between F(v) slopes at
     * v_CE=0 (van Soest & Bobbert, 1993)
     */
    double MP_CE_S_eccentric_;

    /**
     * Eccentric force-velocity relation: Factor by which the force can
     * exceed F_isom for large eccentric velocities (van Soest & Bobbert,
     * 1993)
     */
    double MP_CE_F_eccentric_;
};

}  // namespace hill

}  // namespace pam_models