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

namespace pam_models
{

namespace hill
{

namespace deprecated
{

#ifndef HILLMUSCLE_H_
#define HILLMUSCLE_H_

#include <functional>
#include <vector>

class HillMuscle
{
public:
    HillMuscle(std::string parameterfile, double a_init, double MP_l_MTC_init);
    double get_mucle_tendon_force(double l_MTC,
                                  double dot_l_MTC,
                                  double a,
                                  double l_CE);
    double get_dot_l_CE(double l_MTC, double dot_l_MTC, double a, double l_CE);

private:
    void init_muscle(double f_max, std::vector<double>* hill_params);
    void recalculate_muscle_tendon_force_and_dot_l_CE(double l_MTC,
                                                      double dot_l_MTC,
                                                      double a,
                                                      double l_CE);
    static double brents_fun(
        std::function<double(double)> f,
        double lower,
        double upper,
        double tol,
        unsigned int max_iter);  // find zero point of function
    double F_sum_init_muscle_force_equilib(double l_CE);
    double find_l_CE_init();  // find lCE init according to equilibrium of
                              // forces

    double MP_l_MTC_init;  // initial length of MTC unit, current length =
                           // intial length + length from mujoco
    double MP_l_CE_init;  // initial length of CE, current length = intial
                          // length + length from mujoco
    double a_init;  // initial control signal for muscle tendon unit

    bool F_MTU_current_and_dot_l_CE_current_exist = false;
    double F_MTU_current;
    double dot_l_CE_current;

    double l_MTC_last_recalc;
    double dot_l_MTC_last_recalc;
    double a_last_recalc;
    double l_CE_last_recalc;

    // parameters from Haeufle et al.

    // contractile element (CE)
    //===========================
    double MP_CE_F_max;  // F_max in [N] for Extensor (Kistemaker et al., 2006)
    double MP_CE_l_CEopt;          // optimal length of CE in [m] for Extensor
                                   // (Kistemaker et al., 2006)
    double MP_CE_DeltaW_limb_des;  // width of normalized bell curve in
                                   // descending branch (Moerl et al., 2012)
    double MP_CE_DeltaW_limb_asc;  // width of normalized bell curve in
                                   // ascending branch (Moerl et al., 2012)
    double MP_CE_v_CElimb_des;  // exponent for descending branch (Moerl et al.,
                                // 2012)
    double MP_CE_v_CElimb_asc;  // exponent for ascending branch (Moerl et al.,
                                // 2012)
    double MP_CE_A_rel0;  // parameter for contraction dynamics: maximum value
                          // of A_rel (Guenther, 1997, S. 82)
    double MP_CE_B_rel0;  // parameter for contraction dynmacis: maximum value
                          // of B_rel (Guenther, 1997, S. 82)
    // eccentric force-velocity relation:
    double MP_CE_S_eccentric;  // relation between F(v) slopes at v_CE=0 (van
                               // Soest & Bobbert, 1993)
    double MP_CE_F_eccentric;  // factor by which the force can exceed F_isom
                               // for large eccentric velocities (van Soest &
                               // Bobbert, 1993)

    // paralel elastic element (PEE)
    //===============================

    double MP_PEE_L_PEE0;  // rest length of PEE normalized to optimal lenght of
                           // CE (Guenther et al., 2007)
    double MP_PEE_l_PEE0;  // rest length of PEE (Guenther et al., 2007)
    double MP_PEE_v_PEE;   // exponent of F_PEE (Moerl et al., 2012)
    double MP_PEE_F_PEE;  // force of PEE if l_CE is stretched to deltaWlimb_des
                          // (Moerl et al., 2012)
    double MP_PEE_K_PEE;
    // factor of non-linearity in F_PEE (Guenther et al., 2007)

    // serial damping element (SDE)
    //=============================
    double MP_SDE_D_SE;  // xxx dimensionless factor to scale d_SEmax (Moerl et
                         // al., 2012)
    double MP_SDE_R_SE;  // minimum value of d_SE normalised to d_SEmax (Moerl
                         // et al., 2012)
    double MP_SDE_d_SEmax;
    // maximum value in d_SE in [Ns/m] (Moerl et al., 2012)

    // serial elastic element (SEE)
    // ============================
    double
        MP_SEE_l_SEE0;  // rest length of SEE in [m] (Kistemaker et al., 2006)
    double MP_SEE_DeltaU_SEEnll;  // relativ stretch at non-linear linear
                                  // transition (Moerl et al., 2012)
    double MP_SEE_DeltaU_SEEl;  // relativ additional stretch in the linear part
                                // providing a force increase of deltaF_SEE0
                                // (Moerl, 2012)
    double MP_SEE_DeltaF_SEE0;  // both force at the transition and force
                                // increase in the linear part in [N] (~ 40// of
                                // the maximal isometric muscle force)

    double MP_SEE_l_SEEnll;
    double MP_SEE_v_SEE;
    double MP_SEE_KSEEnl;
    double MP_SEE_KSEEl;
};
}
}
}

#endif
