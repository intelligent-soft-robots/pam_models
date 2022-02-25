#include "pam_models/hill/brents.hpp"

namespace pam_models
{
namespace hill
{
double brents(std::function<double(double)> f,
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

        throw std::runtime_error(
            "pam_models | hills | brents_fun : root is not bracketed");
    }

    // if magnitude of f(lower_bound) is less than magnitude of f(upper_bound)
    if (std::abs(fa) < std::abs(b))
    {
        std::swap(a, b);
        std::swap(fa, fb);
    }

    // c now equals the largest magnitude of the lower and upper bounds
    double c = a;
    // precompute function evalutation for point c by assigning it the same
    // value as fa
    double fc = fa;
    // boolean flag used to evaluate if statement later on
    bool mflag = true;
    // Our Root that will be returned
    double s = 0;
    // Only used if mflag is unset (mflag == false)
    double d = 0;

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

        // checks to see whether we can use the faster converging quadratic
        // && secant methods or if we need to use bisection
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

        // calculate fs
        fs = f(s);
        // first time d is being used (wasnt used on first iteration because
        // mflag was set)
        d = c;
        // set c equal to upper bound
        c = b;
        // set f(c) = f(b)
        fc = fb;

        // fa and fs have opposite signs
        if (fa * fs < 0)
        {
            b = s;
            fb = fs;  // set f(b) = f(s)
        }
        else
        {
            a = s;
            fa = fs;  // set f(a) = f(s)
        }

        // if magnitude of fa is less than magnitude of fb
        if (std::abs(fa) < std::abs(fb))
        {
            // swap a and b
            std::swap(a, b);
            // make sure f(a) and f(b) are correct after swap
            std::swap(fa, fb);
        }

    }  // end for

    // The solution does not converge or iterations are not sufficient
    return -1;
}

}  // namespace hill

}  // namespace pam_models