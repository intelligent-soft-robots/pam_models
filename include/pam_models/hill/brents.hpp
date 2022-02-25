#pragma once

#include <functional>
#include <stdexcept>

namespace pam_models
{
namespace hill
{
/**
 * @brief Brent's method
 *
 * Finds and returns roots of given function with Brent's method.
 *
 * @details Brent's Method uses a combination of the bisection method, inverse
 *          quadratic interpolation and the secant method to find roots.
 *
 * @param f Input function
 * @param lower Lower bound for root search
 * @param upper Upper bound for root search
 * @param tol Allowed tolerance for converging solution
 * @param max_iter Allowed maximum interation steps
 * @return Root value or -1 if solution is not converging within max_iter
 *
 * @cite RosettaCode https://rosettacode.org/wiki/Roots_of_a_function
 */
double brents(std::function<double(double)> f,
              double lower,
              double upper,
              double tol = 0.00000001,
              unsigned int max_iter = 1000);

}  // namespace hill

}  // namespace pam_models