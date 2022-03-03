/**
This class calculates the force of a muscle tendon complex and
and the derivative of the length of the contractile element,
depending on the length of the contractile element, the mtc length,
mtc contraction velocity and activity.
To use it, you have to integrate the lenght of the contractile element
yourself along the simulation.

Implemented by S. Guist, D. Buechler
The code follows the MATLAB implementation by Haeufle et al.


Copyright notice from the MATLAB model by Haeufle et al.:

% Copyright (c) 2014 belongs to D. Haeufle, M. Guenther, A. Bayer, and S.
% Schmitt
% All rights reserved.
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%  1 Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer.
%  2 Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the distribution.
%  3 Neither the name of the owner nor the names of its contributors may be
%    used to endorse or promote products derived from this software without
%    specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
% THE POSSIBILITY OF SUCH DAMAGE.

**/

// clang-format off
// there is some issue here that makes the build break if the order of
// includes is wrong.
#include "stdio.h"
#include "math.h"
#include <cmath>
#include <algorithm>
#include <functional>
#include <iostream>
#include "json_helper/json_helper.hpp"
#include "pam_models/hill/deprecated/deprecated_muscle.hpp"
// clang-format on

namespace pam_models
{
namespace hill
{
namespace deprecated
{
HillMuscle::HillMuscle(std::string parameterfile,
                       double a_init,
                       double l_MTC_change_init)
{
    double f_max;
    std::vector<double> hill_params;
    json_helper::Jsonhelper jh(parameterfile);
    f_max = jh.j["f_max"];
    this->MP_l_MTC_init = (double)jh.j["length"] + l_MTC_change_init;

    jh.json2vector("hill_params", hill_params);

    init_muscle(f_max, &hill_params);

    this->a_init = a_init;
    this->MP_l_CE_init = find_l_CE_init();
    // printf("lce_init: %f\n", this->MP_l_CE_init);*/
}

void HillMuscle::init_muscle(double f_max, std::vector<double>* hill_params)
{
    // parameters from Haeufle et al.

    // contractile element (CE)
    //===========================

    MP_CE_F_max = f_max;  // F_max in [N] for Extensor (Kistemaker et al., 2006)
                          // --- set fixed
    MP_CE_l_CEopt =
        (*hill_params)[0];  // optimal length of CE in [m] for Extensor
                            // (Kistemaker et al., 2006) --- learn two values
    MP_CE_DeltaW_limb_des =
        (*hill_params)[1];  // width of normalized bell curve in descending
                            // branch (Moerl et al., 2012) --- learn one value
    MP_CE_DeltaW_limb_asc =
        (*hill_params)[2];  // width of normalized bell curve in ascending
                            // branch (Moerl et al., 2012) --- learn one value
    MP_CE_v_CElimb_des =
        (*hill_params)[3];  // exponent for descending branch (Moerl et al.,
                            // 2012) --- learn two values
    MP_CE_v_CElimb_asc =
        (*hill_params)[4];  // exponent for ascending branch (Moerl et al.,
                            // 2012) --- learn two values
    MP_CE_A_rel0 = (*hill_params)[5];  // parameter for contraction dynamics:
                                       // maximum value of A_rel (Guenther,
                                       // 1997, S. 82) --- learn two values
    MP_CE_B_rel0 = (*hill_params)[6];  // parameter for contraction dynmacis:
                                       // maximum value of B_rel (Guenther,
                                       // 1997, S. 82) --- learn two values
    // eccentric force-velocity relation:
    MP_CE_S_eccentric =
        (*hill_params)[7];  // relation between F(v) slopes at v_CE=0 (van Soest
                            // & Bobbert, 1993) --- learn two values
    MP_CE_F_eccentric =
        (*hill_params)[8];  // factor by which the force can exceed F_isom for
                            // large eccentric velocities (van Soest & Bobbert,
                            // 1993) --- learn two values

    // paralel elastic element (PEE)
    //===============================

    MP_PEE_L_PEE0 =
        (*hill_params)[9];  // rest length of PEE normalized to optimal lenght
                            // of CE (Guenther et al., 2007) --- keep
    MP_PEE_l_PEE0 =
        MP_PEE_L_PEE0 *
        MP_CE_l_CEopt;  // rest length of PEE (Guenther et al., 2007)
    MP_PEE_v_PEE =
        (*hill_params)[10];  // exponent of F_PEE (Moerl et al., 2012) --- keep
    MP_PEE_F_PEE = (*hill_params)[11];  // force of PEE if l_CE is stretched to
                                        // deltaWlimb_des (Moerl et al., 2012)
                                        // --- learn one value
    MP_PEE_K_PEE =
        MP_PEE_F_PEE *
        (MP_CE_F_max /
         pow(MP_CE_l_CEopt * (MP_CE_DeltaW_limb_des + 1 - MP_PEE_L_PEE0),
             MP_PEE_v_PEE));
    // factor of non-linearity in F_PEE (Guenther et al., 2007)

    // serial damping element (SDE)
    //=============================
    MP_SDE_D_SE = (*hill_params)[12];  // xxx dimensionless factor to scale
                                       // d_SEmax (Moerl et al., 2012) --- keep
    MP_SDE_R_SE = (*hill_params)[13];  // minimum value of d_SE normalised to
                                       // d_SEmax (Moerl et al., 2012) --- keep
    MP_SDE_d_SEmax = MP_SDE_D_SE * (MP_CE_F_max * MP_CE_A_rel0) /
                     (MP_CE_l_CEopt * MP_CE_B_rel0);
    // maximum value in d_SE in [Ns/m] (Moerl et al., 2012)

    // serial elastic element (SEE)
    // ============================
    MP_SEE_l_SEE0 =
        (*hill_params)[14];  // rest length of SEE in [m] (Kistemaker et al.,
                             // 2006) --- learn two values
    MP_SEE_DeltaU_SEEnll =
        (*hill_params)[15];  // relativ stretch at non-linear linear transition
                             // (Moerl et al., 2012) --- keep
    MP_SEE_DeltaU_SEEl =
        (*hill_params)[16];  // relativ additional stretch in the linear part
                             // providing a force increase of deltaF_SEE0
                             // (Moerl, 2012) --- keep
    MP_SEE_DeltaF_SEE0 =
        (*hill_params)[17];  // both force at the transition and force increase
                             // in the linear part in [N] (~ 40// of the maximal
                             // isometric muscle force) --- keep, set by
                             // iesometric force -> 480

    MP_SEE_l_SEEnll = (1 + MP_SEE_DeltaU_SEEnll) * MP_SEE_l_SEE0;
    MP_SEE_v_SEE = MP_SEE_DeltaU_SEEnll / MP_SEE_DeltaU_SEEl;
    MP_SEE_KSEEnl = MP_SEE_DeltaF_SEE0 /
                    pow(MP_SEE_DeltaU_SEEnll * MP_SEE_l_SEE0, MP_SEE_v_SEE);
    MP_SEE_KSEEl = MP_SEE_DeltaF_SEE0 / (MP_SEE_DeltaU_SEEl * MP_SEE_l_SEE0);
}

double HillMuscle::get_mucle_tendon_force(double l_MTC,
                                          double dot_l_MTC,
                                          double a,
                                          double l_CE)
{
    if (!F_MTU_current_and_dot_l_CE_current_exist ||
        (l_MTC != l_MTC_last_recalc || dot_l_MTC != dot_l_MTC_last_recalc ||
         a != a_last_recalc || l_CE != l_CE_last_recalc))
    {
        recalculate_muscle_tendon_force_and_dot_l_CE(l_MTC, dot_l_MTC, a, l_CE);
        F_MTU_current_and_dot_l_CE_current_exist = true;
    }
    return F_MTU_current;
}

double HillMuscle::get_dot_l_CE(double l_MTC,
                                double dot_l_MTC,
                                double a,
                                double l_CE)
{
    if (!F_MTU_current_and_dot_l_CE_current_exist ||
        (l_MTC != l_MTC_last_recalc || dot_l_MTC != dot_l_MTC_last_recalc ||
         a != a_last_recalc || l_CE != l_CE_last_recalc))
    {
        recalculate_muscle_tendon_force_and_dot_l_CE(l_MTC, dot_l_MTC, a, l_CE);
        F_MTU_current_and_dot_l_CE_current_exist = true;
    }
    return dot_l_CE_current;
}

void HillMuscle::recalculate_muscle_tendon_force_and_dot_l_CE(double l_MTC,
                                                              double dot_l_MTC,
                                                              double a,
                                                              double l_CE)
{
    l_MTC_last_recalc = l_MTC;
    dot_l_MTC_last_recalc = dot_l_MTC;
    a_last_recalc = a;
    l_CE_last_recalc = l_CE;

    l_MTC = l_MTC + MP_l_MTC_init;
    l_CE = l_CE + MP_l_CE_init;
    double l_SE = l_MTC - l_CE;

    double F_isom = 0;
    if (MP_CE_l_CEopt <= l_CE)
        F_isom +=
            (exp(-pow(abs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_des),
                      MP_CE_v_CElimb_des)));
    if (MP_CE_l_CEopt > l_CE && MP_CE_l_CEopt > l_CE)
        F_isom +=
            (exp(-pow(abs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_asc),
                      MP_CE_v_CElimb_asc)));

    double F_PEE = 0;
    if (l_CE >= MP_PEE_l_PEE0)
    {
        F_PEE = MP_PEE_K_PEE * pow(l_CE - MP_PEE_l_PEE0, MP_PEE_v_PEE);
    }

    double F_SEE =
        (l_SE >= MP_SEE_l_SEEnll) *
        (MP_SEE_DeltaF_SEE0 + MP_SEE_KSEEl * (l_SE - MP_SEE_l_SEEnll));
    if (l_SE > MP_SEE_l_SEE0 && l_SE < MP_SEE_l_SEEnll)
        F_SEE += MP_SEE_KSEEnl * (pow(l_SE - MP_SEE_l_SEE0, MP_SEE_v_SEE));

    double A_rel =
        (1.0 * (l_CE < MP_CE_l_CEopt) + F_isom * (l_CE >= MP_CE_l_CEopt)) *
        MP_CE_A_rel0 * 1 / 4 * (1 + 3 * a);

    double B_rel = MP_CE_B_rel0 * 1 * 1 / 7 * (3 + 4 * a);

    double D_0 =
        MP_CE_l_CEopt * B_rel * MP_SDE_d_SEmax *
        (MP_SDE_R_SE + (1 - MP_SDE_R_SE) * (a * F_isom + F_PEE / MP_CE_F_max));

    double C_2 = MP_SDE_d_SEmax * (MP_SDE_R_SE - (A_rel - F_PEE / MP_CE_F_max) *
                                                     (1 - MP_SDE_R_SE));

    double C_1 =
        -(C_2 * dot_l_MTC + D_0 + F_SEE - F_PEE + (MP_CE_F_max * A_rel));

    double C_0 =
        D_0 * dot_l_MTC +
        MP_CE_l_CEopt * B_rel * (F_SEE - F_PEE - MP_CE_F_max * a * F_isom);

    double dot_l_CE =
        (-C_1 - sqrt(C_1 * C_1 - 4 * C_2 * C_0)) /
        (2 * C_2);  // quadratic equation for concentric contractions (-sqrt)

    // printf("hillMuscle -dot_l_MTC:%f -B_rel:%f -a:%f -F_isom:%f -F_PEE:%f
    // -F_SEE:%f  -D_0:%f\n", dot_l_MTC, B_rel, a, F_isom, F_PEE, F_SEE, D_0);
    // printf("hillMuscle -C0:%f -C1:%f -C2: %f\n", C_0, C_1, C_2);
    // printf("hillMuscle-dot_l_CE: %f\n", dot_l_CE);

    if (dot_l_CE >
        0)  // dot_l_CE > 0 -> eccentric contraction -> recalculate dot_l_CE
    {
        double A_rel_con = A_rel;
        double B_rel_con = B_rel;
        A_rel = -MP_CE_F_eccentric * a * F_isom;
        B_rel = a * F_isom * (1 - MP_CE_F_eccentric) /
                (a * F_isom + A_rel_con) * B_rel_con / MP_CE_S_eccentric;
        D_0 = MP_CE_l_CEopt * B_rel * MP_SDE_d_SEmax *
              (MP_SDE_R_SE +
               (1 - MP_SDE_R_SE) * (a * F_isom + F_PEE / MP_CE_F_max));
        C_2 = MP_SDE_d_SEmax *
              (MP_SDE_R_SE - (A_rel - F_PEE / MP_CE_F_max) * (1 - MP_SDE_R_SE));
        C_1 = -(C_2 * dot_l_MTC + D_0 + F_SEE - F_PEE + (MP_CE_F_max * A_rel));
        C_0 = D_0 * dot_l_MTC + MP_CE_l_CEopt * B_rel *
                                    (F_SEE - F_PEE - MP_CE_F_max * a * F_isom);
        dot_l_CE =
            (-C_1 + sqrt(C_1 * C_1 - 4 * C_2 * C_0)) /
            (2 * C_2);  // quadratic equation for eccentric contractions (+sqrt)
        // printf("hillMuscle-dot_l_CE: %f (recalculated)\n", dot_l_CE);
    }

    dot_l_CE_current = dot_l_CE;

    double F_CE =
        MP_CE_F_max *
        (((a * F_isom + A_rel) / (1 - (dot_l_CE / (B_rel * MP_CE_l_CEopt)))) -
         A_rel);

    double F_SDE =
        MP_SDE_d_SEmax *
        ((1 - MP_SDE_R_SE) * ((F_CE + F_PEE) / MP_CE_F_max) + MP_SDE_R_SE) *
        (dot_l_MTC - dot_l_CE);

    F_MTU_current = F_SEE + F_SDE;
}

double HillMuscle::find_l_CE_init()
{
    std::function<double(double)> f(
        std::bind(&HillMuscle::F_sum_init_muscle_force_equilib,
                  this,
                  std::placeholders::_1));
    return brents_fun(f, 0, MP_l_MTC_init, 0.00000001, 1000);
}

double HillMuscle::F_sum_init_muscle_force_equilib(double l_CE)
{
    double l_MTC = MP_l_MTC_init;

    double F_isom, F_PEE, F_SEE;

    // Isometric force (Force length relation)
    // Guenther et al. 2007
    if (l_CE >= MP_CE_l_CEopt)  // descending branch
        F_isom =
            exp(-pow(abs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_des),
                     MP_CE_v_CElimb_des));
    else  // ascending branch
        F_isom =
            exp(-pow(abs(((l_CE / MP_CE_l_CEopt) - 1) / MP_CE_DeltaW_limb_asc),
                     MP_CE_v_CElimb_asc));

    // Force of the parallel elastic element
    if (l_CE >= MP_PEE_l_PEE0)
        F_PEE = MP_PEE_K_PEE * pow(l_CE - MP_PEE_l_PEE0, MP_PEE_v_PEE);
    else  // shorter than slack length
        F_PEE = 0;

    // Force of the serial elastic element
    double l_SEE = abs(l_MTC - l_CE);
    if ((l_SEE > MP_SEE_l_SEE0) && (l_SEE < MP_SEE_l_SEEnll))  // non-linear
                                                               // part
        F_SEE = MP_SEE_KSEEnl * pow(l_SEE - MP_SEE_l_SEE0, MP_SEE_v_SEE);
    else if (l_SEE >= MP_SEE_l_SEEnll)  // linear part
        F_SEE = MP_SEE_DeltaF_SEE0 + MP_SEE_KSEEl * (l_SEE - MP_SEE_l_SEEnll);
    else  // salck length
        F_SEE = 0;

    // Contractile element force (isometric)
    double F_CE = MP_CE_F_max * a_init * F_isom;

    double F_sum = F_SEE - F_CE - F_PEE;

    return F_sum;
}

// source: RosettaCode  https://rosettacode.org/wiki/Roots_of_a_function
double HillMuscle::brents_fun(std::function<double(double)> f,
                              double lower,
                              double upper,
                              double tol,
                              unsigned int max_iter)
{
    double a = lower;
    double b = upper;
    double fa = f(a);  // calculated now to save function calls
    double fb = f(b);  // calculated now to save function calls
    double fs = 0;     // initialize

    if (!(fa * fb < 0))
    {
        if (fb == 0) return b;
        if (fa == 0) return a;
        // std::cout << "Signs of f(lower_bound):" << fa <<" and
        // f(upper_bound):" << fb <<" must be opposites" << std::endl; // throws
        // exception if root isn't bracketed
        return -11;
    }

    if (std::abs(fa) < std::abs(b))  // if magnitude of f(lower_bound) is less
                                     // than magnitude of f(upper_bound)
    {
        std::swap(a, b);
        std::swap(fa, fb);
    }

    double c =
        a;  // c now equals the largest magnitude of the lower and upper bounds
    double fc = fa;  // precompute function evalutation for point c by assigning
                     // it the same value as fa
    bool mflag = true;  // boolean flag used to evaluate if statement later on
    double s = 0;       // Our Root that will be returned
    double d = 0;       // Only used if mflag is unset (mflag == false)

    for (unsigned int iter = 1; iter < max_iter; ++iter)
    {
        // stop if converged on root or error is less than tolerance
        if (std::abs(b - a) < tol)
        {
            return s;
        }  // end if

        if (fa != fc && fb != fc)
        {
            // use inverse quadratic interopolation
            s = (a * fb * fc / ((fa - fb) * (fa - fc))) +
                (b * fa * fc / ((fb - fa) * (fb - fc))) +
                (c * fa * fb / ((fc - fa) * (fc - fb)));
        }
        else
        {
            // secant method
            s = b - fb * (b - a) / (fb - fa);
        }

        // checks to see whether we can use the faster converging quadratic &&
        // secant methods or if we need to use bisection
        if (((s < (3 * a + b) * 0.25) || (s > b)) ||
            (mflag && (std::abs(s - b) >= (std::abs(b - c) * 0.5))) ||
            (!mflag && (std::abs(s - b) >= (std::abs(c - d) * 0.5))) ||
            (mflag && (std::abs(b - c) < tol)) ||
            (!mflag && (std::abs(c - d) < tol)))
        {
            // bisection method
            s = (a + b) * 0.5;

            mflag = true;
        }
        else
        {
            mflag = false;
        }

        fs = f(s);  // calculate fs
        d = c;      // first time d is being used (wasnt used on first iteration
                    // because mflag was set)
        c = b;      // set c equal to upper bound
        fc = fb;    // set f(c) = f(b)

        if (fa * fs < 0)  // fa and fs have opposite signs
        {
            b = s;
            fb = fs;  // set f(b) = f(s)
        }
        else
        {
            a = s;
            fa = fs;  // set f(a) = f(s)
        }

        if (std::abs(fa) <
            std::abs(fb))  // if magnitude of fa is less than magnitude of fb
        {
            std::swap(a, b);  // swap a and b
            std::swap(fa,
                      fb);  // make sure f(a) and f(b) are correct after swap
        }

    }  // end for

    // std::cout<< "The solution does not converge or iterations are not
    // sufficient" << std::endl;

    return -1;
}

}  // namespace deprecated
}  // namespace hill
}  // namespace pam_models
