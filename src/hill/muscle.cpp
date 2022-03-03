#include "pam_models/hill/muscle.hpp"

namespace pam_models
{
namespace hill
{
double init_muscle_force_equilibrium(double l_CE,
                                     const ParallelElasticElement& PEE,
                                     const ContractileElement& CE,
                                     const SerialElasticElement& SEE,
                                     double a_init,
                                     double MP_l_MTC_init)

{
    double l_MTC = MP_l_MTC_init;

    // Isometric force (Force length relation)
    // Guenther et al. 2007
    double F_isom;
    if (l_CE >= CE.MP_CE_l_CEopt_)  // descending branch
    {
        F_isom = exp(-pow(
            abs(((l_CE / CE.MP_CE_l_CEopt_) - 1) / CE.MP_CE_DeltaW_limb_des_),
            CE.MP_CE_v_CElimb_des_));
    }
    else  // ascending branch
    {
        F_isom = exp(-pow(
            abs(((l_CE / CE.MP_CE_l_CEopt_) - 1) / CE.MP_CE_DeltaW_limb_asc_),
            CE.MP_CE_v_CElimb_asc_));
    }

    // Force of the parallel elastic element
    double F_PEE;
    if (l_CE >= PEE.MP_PEE_l_PEE0_)
    {
        F_PEE = PEE.MP_PEE_K_PEE_ *
                pow(l_CE - PEE.MP_PEE_l_PEE0_, PEE.MP_PEE_v_PEE_);
    }
    else  // shorter than slack length
    {
        F_PEE = 0;
    }

    // Force of the serial elastic element
    double F_SEE;
    double l_SEE = abs(l_MTC - l_CE);
    if ((l_SEE > SEE.l_0_) && (l_SEE < SEE.l_nll_))  // non-linear part
    {
        F_SEE = SEE.k_nl_ * pow(l_SEE - SEE.l_0_, SEE.v_);
    }
    else if (l_SEE >= SEE.l_nll_)  // linear part
    {
        F_SEE = SEE.deltaF_0_ + SEE.k_l_ * (l_SEE - SEE.l_nll_);
    }
    else  // slack length
    {
        F_SEE = 0;
    }

    // Contractile element force (isometric)
    double F_CE = CE.MP_CE_F_max_ * a_init * F_isom;

    double F_sum = F_SEE - F_CE - F_PEE;

    return F_sum;
}

Muscle::Muscle(ContractileElement contractile,
               ParallelElasticElement parallel_elastic,
               SerialDampingElement serial_damping,
               SerialElasticElement serial_elastic,
               double a_init,
               double l_MTC_change_init,
               double length)
    : contractile_(contractile),
      parallel_elastic_(parallel_elastic),
      serial_damping_(serial_damping),
      serial_elastic_(serial_elastic),
      MP_l_MTC_init_(length + l_MTC_change_init),
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

std::tuple<double, double> Muscle::get(double l_MTC,
                                       double dot_l_MTC,
                                       double a,
                                       double l_CE)
{
    l_MTC = l_MTC + MP_l_MTC_init_;
    l_CE = l_CE + MP_l_CE_init_;

    double F_isom = contractile_.get_isometric_force(l_CE);
    double F_PEE = parallel_elastic_.get_force(l_CE);
    double F_SEE = serial_elastic_.get_force(l_MTC, l_CE);
    double A_rel = contractile_.get_a_relative(l_CE, F_isom, a);
    double B_rel = contractile_.get_b_relative(a);

    double D_0;
    {
        D_0 = contractile_.MP_CE_l_CEopt_ * B_rel *
              serial_damping_.MP_SDE_d_SEmax_;
        D_0 *= (serial_damping_.MP_SDE_R_SE_ +
                (1 - serial_damping_.MP_SDE_R_SE_) *
                    (a * F_isom + F_PEE / contractile_.MP_CE_F_max_));
    }

    double C_2;
    {
        double rel = A_rel - F_PEE / contractile_.MP_CE_F_max_;
        double one_min = 1.0 - serial_damping_.MP_SDE_R_SE_;
        C_2 = serial_damping_.MP_SDE_d_SEmax_ *
              (serial_damping_.MP_SDE_R_SE_ - rel * one_min);
    }

    double C_1;
    {
        C_1 = -(C_2 * dot_l_MTC + D_0 + F_SEE - F_PEE +
                (contractile_.MP_CE_F_max_ * A_rel));
    }

    double C_0;
    {
        double f_s = F_SEE - F_PEE - contractile_.MP_CE_F_max_ * a * F_isom;
        C_0 = D_0 * dot_l_MTC + contractile_.MP_CE_l_CEopt_ * B_rel * f_s;
    }

    // quadratic equation for concentric contractions (-sqrt)
    double dot_l_CE = (-C_1 - sqrt(C_1 * C_1 - 4 * C_2 * C_0)) / (2 * C_2);

    if (dot_l_CE >
        0)  // dot_l_CE > 0 -> eccentric contraction -> recalculate dot_l_CE
    {
        double A_rel_con = A_rel;
        double B_rel_con = B_rel;
        A_rel = -contractile_.MP_CE_F_eccentric_ * a * F_isom;
        B_rel = a * F_isom * (1 - contractile_.MP_CE_F_eccentric_) /
                (a * F_isom + A_rel_con) * B_rel_con /
                contractile_.MP_CE_S_eccentric_;
        D_0 = contractile_.MP_CE_l_CEopt_ * B_rel *
              serial_damping_.MP_SDE_d_SEmax_ *
              (serial_damping_.MP_SDE_R_SE_ +
               (1 - serial_damping_.MP_SDE_R_SE_) *
                   (a * F_isom + F_PEE / contractile_.MP_CE_F_max_));
        C_2 = serial_damping_.MP_SDE_d_SEmax_ *
              (serial_damping_.MP_SDE_R_SE_ -
               (A_rel - F_PEE / contractile_.MP_CE_F_max_) *
                   (1 - serial_damping_.MP_SDE_R_SE_));
        C_1 = -(C_2 * dot_l_MTC + D_0 + F_SEE - F_PEE +
                (contractile_.MP_CE_F_max_ * A_rel));
        C_0 = D_0 * dot_l_MTC +
              contractile_.MP_CE_l_CEopt_ * B_rel *
                  (F_SEE - F_PEE - contractile_.MP_CE_F_max_ * a * F_isom);
        // quadratic equation for eccentric contractions (+sqrt)
        dot_l_CE = (-C_1 + sqrt(C_1 * C_1 - 4 * C_2 * C_0)) / (2 * C_2);
    }

    double dot_l_CE_current = dot_l_CE;

    double F_CE;

    F_CE = contractile_.MP_CE_F_max_;
    F_CE *= ((a * F_isom + A_rel) /
             (1 - (dot_l_CE / (B_rel * contractile_.MP_CE_l_CEopt_)))) -
            A_rel;

    double F_SDE;
    F_SDE = serial_damping_.MP_SDE_d_SEmax_ *
            ((1 - serial_damping_.MP_SDE_R_SE_) *
                 ((F_CE + F_PEE) / contractile_.MP_CE_F_max_) +
             serial_damping_.MP_SDE_R_SE_) *
            (dot_l_MTC - dot_l_CE);

    double F_MTU_current = F_SEE + F_SDE;

    return std::make_tuple(F_MTU_current, dot_l_CE_current);
}

}  // namespace hill

}  // namespace pam_models