#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Fresnel.cpp"
//# include <cstdlib>

# include "pch.h"
# include <cmath>
# include "fresnel.h"

//****************************************************************************80
namespace MazeMap {
    using namespace std;
    void fresnel(double x, double& c, double& s)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel() computes Fresnel integrals C(x) and S(x).
        //
        //  Licensing:
        //
        //    This routine is copyrighted by Shanjie Zhang and Jianming Jin.  However,
        //    they give permission to incorporate this routine into a user program
        //    provided that the copyright is acknowledged.
        //
        //  Modified:
        //
        //    11 July 2025
        //
        //  Author:
        //
        //    Original Fortran77 version by Shanjie Zhang, Jianming Jin.
        //    This version by John Burkardt.
        //
        //  Reference:
        //
        //    John D Cook,
        //    Cornu's spiral,
        //    Posted 23 March 2016.
        //    https://www.johndcook.com/blog/2016/03/23/cornus-spiral/
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    double X, the argument.
        //
        //  Output:
        //
        //    double C, S, the function values.
        //
    {
        double eps;
        double f;
        double f0;
        double f1;
        double g;
        int k;
        int m;
        double pi;
        double px;
        double q;
        double r;
        double su;
        double t;
        double t0;
        double t2;
        double xa;

        eps = 1.0E-15;
        pi = 3.141592653589793;
        xa = fabs(x);
        px = pi * xa;
        t = 0.5 * px * xa;
        t2 = t * t;
        //
        //  x == 0
        //
        if (xa == 0.0)
        {
            c = 0.0;
            s = 0.0;
        }
        //
        //  0 < x < 2.5
        //
        else if (xa < 2.5)
        {
            r = xa;
            c = r;
            for (k = 1; k <= 50; k++)
            {
                r = -0.5 * r * (4.0 * k - 3.0) / k
                    / (2.0 * k - 1.0) / (4.0 * k + 1.0) * t2;
                c = c + r;
                if (fabs(r) < fabs(c) * eps)
                {
                    break;
                }
            }

            s = xa * t / 3.0;
            r = s;
            for (k = 1; k <= 50; k++)
            {
                r = -0.5 * r * (4.0 * k - 1.0) / k
                    / (2.0 * k + 1.0) / (4.0 * k + 3.0) * t2;
                s = s + r;
                if (fabs(r) < fabs(s) * eps)
                {
                    if (x < 0.0)
                    {
                        c = -c;
                        s = -s;
                    }
                    return;
                }
            }
        }
        //
        //  2.5 <= x < 4.5
        //
        else if (xa < 4.5)
        {
            m = (int)(42.0 + 1.75 * t);
            su = 0.0;
            c = 0.0;
            s = 0.0;
            f1 = 0.0;
            f0 = 1.0E-100;

            for (k = m; 0 <= k; k--)
            {
                f = (2.0 * k + 3.0) * f0 / t - f1;
                if (k == (int)(k / 2) * 2)
                {
                    c = c + f;
                }
                else
                {
                    s = s + f;
                }
                su = su + (2.0 * k + 1.0) * f * f;
                f1 = f0;
                f0 = f;
            }

            q = sqrt(su);
            c = c * xa / q;
            s = s * xa / q;
        }
        //
        //  4.5 <= x
        //
        else
        {
            r = 1.0;
            f = 1.0;
            for (k = 1; k <= 20; k++)
            {
                r = -0.25 * r * (4.0 * k - 1.0) * (4.0 * k - 3.0) / t2;
                f = f + r;
            }
            r = 1.0 / (px * xa);
            g = r;
            for (k = 1; k <= 12; k++)
            {
                r = -0.25 * r * (4.0 * k + 1.0) * (4.0 * k - 1.0) / t2;
                g = g + r;
            }

            t0 = t - (int)(t / (2.0 * pi)) * 2.0 * pi;
            c = 0.5 + (f * sin(t0) - g * cos(t0)) / px;
            s = 0.5 - (f * cos(t0) + g * sin(t0)) / px;
        }
        //
        //  Apply symmetry for x < 0.
        //
        if (x < 0.0)
        {
            c = -c;
            s = -s;
        }

        return;
    }
    //****************************************************************************80

    double fresnel_cos(double x)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel_cos() evaluates the Fresnel cosine integral C(x).
        //
        //  Licensing:
        //
        //    This code is distributed under the MIT license.
        //
        //  Modified:
        //
        //    12 July 2025
        //
        //  Author:
        //
        //    John Burkardt.
        //
        //  Reference:
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    real X: the argument.
        //
        //  Output:
        //
        //    real C: the Fresnel cosine integral value at X.
        //
    {
        double c;
        double s;

        fresnel(x, c, s);

        return c;
    }
    //****************************************************************************80

    double fresnel_sin(double x)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel_sin() evaluates the Fresnel sine integral S(x).
        //
        //  Licensing:
        //
        //    This code is distributed under the MIT license.
        //
        //  Modified:
        //
        //    12 July 2025
        //
        //  Author:
        //
        //    John Burkardt.
        //
        //  Reference:
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    real X: the argument.
        //
        //  Output:
        //
        //    real S: the Fresnel sine integral value at X.
        //
    {
        double c;
        double s;

        fresnel(x, c, s);

        return s;
    }


#if FRESNEL_PRECISION == 4
    // Normally 50
    constexpr int ITERATIONS_A = 50;
    constexpr float BASE_ITERATIONS_MID = 17.73585f;
    constexpr float ITERATION_SCALE_MID = 1.85f;
    constexpr float INITIAL_F0 = 1.175494e-38f;
    // Normally 20
    constexpr int ITERATIONS_B = 6;
    // Normally 12
    constexpr int ITERATIONS_C = 4;
    constexpr float EPSILON_FRESNEL = 9.0E-4f;
#endif
#if FRESNEL_PRECISION == 5
    // Normally 50
    constexpr int ITERATIONS_A = 50;
    constexpr float BASE_ITERATIONS_MID = 15.39205f;
    constexpr float ITERATION_SCALE_MID = 2.3f;
    constexpr float INITIAL_F0 = 1.175494e-38f;
    // Normally 20
    constexpr int ITERATIONS_B = 6;
    // Normally 12
    constexpr int ITERATIONS_C = 4;
    constexpr float EPSILON_FRESNEL = 1.0E-4f;
#endif
    //****************************************************************************80


    void fresnelf(float x, float& c, float& s)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel() computes Fresnel integrals C(x) and S(x).
        //
        //  Licensing:
        //
        //    This routine is copyrighted by Shanjie Zhang and Jianming Jin.  However,
        //    they give permission to incorporate this routine into a user program
        //    provided that the copyright is acknowledged.
        //
        //  Modified:
        //
        //    11 July 2025
        //
        //  Author:
        //
        //    Original Fortran77 version by Shanjie Zhang, Jianming Jin.
        //    This version by John Burkardt.
        //
        //  Reference:
        //
        //    John D Cook,
        //    Cornu's spiral,
        //    Posted 23 March 2016.
        //    https://www.johndcook.com/blog/2016/03/23/cornus-spiral/
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    double X, the argument.
        //
        //  Output:
        //
        //    double C, S, the function values.
        //
    {
        float f;
        float f0;
        float f1;
        float g;
        int k;
        int m;
        float px;
        float q;
        float su;
        float t;
        float t0;
        float t2;
        float xa;

        xa = fabsf(x);
        px = PI_F * xa;
        t = 0.5f * px * xa;
        t2 = t * t;
        //
        //  x == 0
        //
        if (xa == 0.0f)
        {
            c = 0.0f;
            s = 0.0f;
            return;
        }
        //
        //  0 < x < 2.5
        //
        if (xa < FIRST_BOUNDARY)
        {
            float r = xa;
            float r2 = 0.0f;
            c = r;
            for (k = 1; k <= ITERATIONS_A; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.5f * r * ((4.0f * kf) - 3.0f) / kf
                    / ((2.0f * kf) - 1.0f) / ((4.0f * kf) + 1.0f) * t2;
                if (c + r == c)
                {
                    r2 += r;
                }
                else
                {
                    c = c + r + r2;
                    r2 = 0.0f;
                }
                if (fabsf(r) < fabsf(c) * EPSILON_FRESNEL)
                {
                    c = c + r2 + r;
                    break;
                }
            }

            s = xa * t / 3.0f;
            r = s;
            for (k = 1; k <= ITERATIONS_A; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.5f * r * ((4.0f * kf) - 1.0f) / kf
                    / ((2.0f * kf) + 1.0f) / ((4.0f * kf) + 3.0f) * t2;
                s = s + r;
                if (fabsf(r) < fabsf(s) * EPSILON_FRESNEL)
                {
                    if (x < 0.0f)
                    {
                        c = -c;
                        s = -s;
                    }
                    return;
                }
            }
        }
        //
        //  2.5 <= x < 4.5
        //
        else if (xa < SECOND_BOUNDARY)
        {
            //m = (int)(42.0f + 1.75f * t);
            m = (int)(BASE_ITERATIONS_MID + ITERATION_SCALE_MID * t);
            //m = (int)(14.314*x);
            su = 0.0f;
            c = 0.0f;
            s = 0.0f;
            f1 = 0.0f;
            f0 = INITIAL_F0;

            for (k = m; 0 <= k; k--)
            {
                const float kf = static_cast<float>(k);
                f = ((2.0f * kf) + 3.0f) * f0 / t - f1;
                if ((k % 2) == 0)
                {
                    c = c + f;
                }
                else
                {
                    s = s + f;
                }
                su = su + (((2.0f * kf) + 1.0f) * f * f);
                f1 = f0;
                f0 = f;
            }

            q = sqrtf(su);
            c = c * xa / q;
            s = s * xa / q;
        }
        //
        //  4.5 <= x
        //
        else
        {
            float r = 1.0f;
            f = 1.0f;
            for (k = 1; k <= ITERATIONS_B; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.25f * r * ((4.0f * kf) - 1.0f) * ((4.0f * kf) - 3.0f) / t2;
                f = f + r;
            }
            r = 1.0f / (px * xa);
            g = r;
            for (k = 1; k <= ITERATIONS_C; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.25f * r * ((4.0f * kf) + 1.0f) * ((4.0f * kf) - 1.0f) / t2;
                g = g + r;
            }

            const float wrappedTurns = static_cast<float>(static_cast<int>(t / (2.0f * PI_F)));
            t0 = t - (wrappedTurns * 2.0f * PI_F);
            c = 0.5f + (f * sinf(t0) - g * cosf(t0)) / px;
            s = 0.5f - (f * cosf(t0) + g * sinf(t0)) / px;
        }
        //
        //  Apply symmetry for x < 0.
        //
        if (x < 0.0f)
        {
            c = -c;
            s = -s;
        }

        return;
    }

    void fresnelf_test(float x, float& c, float& s, float iterations)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel() computes Fresnel integrals C(x) and S(x).
        //
        //  Licensing:
        //
        //    This routine is copyrighted by Shanjie Zhang and Jianming Jin.  However,
        //    they give permission to incorporate this routine into a user program
        //    provided that the copyright is acknowledged.
        //
        //  Modified:
        //
        //    11 July 2025
        //
        //  Author:
        //
        //    Original Fortran77 version by Shanjie Zhang, Jianming Jin.
        //    This version by John Burkardt.
        //
        //  Reference:
        //
        //    John D Cook,
        //    Cornu's spiral,
        //    Posted 23 March 2016.
        //    https://www.johndcook.com/blog/2016/03/23/cornus-spiral/
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    double X, the argument.
        //
        //  Output:
        //
        //    double C, S, the function values.
        //
    {
        float f;
        float f0;
        float f1;
        float g;
        int k;
        int m;
        float px;
        float q;
        float r;
        float su;
        float t;
        float t0;
        float t2;
        float xa;

        xa = fabsf(x);
        px = PI_F * xa;
        t = 0.5f * px * xa;
        t2 = t * t;
        //
        //  x == 0
        //
        if (xa == 0.0f)
        {
            c = 0.0f;
            s = 0.0f;
        }
        //
        //  0 < x < 2.5
        //
        else if (xa < FIRST_BOUNDARY)
        {
            r = xa;
            c = r;
            for (k = 1; k <= ITERATIONS_A; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.5f * r * ((4.0f * kf) - 3.0f) / kf
                    / ((2.0f * kf) - 1.0f) / ((4.0f * kf) + 1.0f) * t2;
                c = c + r;
                if (fabsf(r) < fabsf(c) * EPSILON_FRESNEL)
                {
                    break;
                }
            }

            s = xa * t / 3.0f;
            r = s;
            for (k = 1; k <= ITERATIONS_A; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.5f * r * ((4.0f * kf) - 1.0f) / kf
                    / ((2.0f * kf) + 1.0f) / ((4.0f * kf) + 3.0f) * t2;
                s = s + r;
                if (fabsf(r) < fabsf(s) * EPSILON_FRESNEL)
                {
                    if (x < 0.0f)
                    {
                        c = -c;
                        s = -s;
                    }
                    return;
                }
            }
        }
        //
        //  2.5 <= x < 4.5
        //
        else if (xa < SECOND_BOUNDARY)
        {
            //m = (int)(42.0f + 1.75f * t);
            m = (int)(iterations);
            su = 0.0f;
            c = 0.0f;
            s = 0.0f;
            f1 = 0.0f;
            f0 = INITIAL_F0;

            for (k = m; 0 <= k; k--)
            {
                const float kf = static_cast<float>(k);
                f = ((2.0f * kf) + 3.0f) * f0 / t - f1;
                if ((k % 2) == 0)
                {
                    c = c + f;
                }
                else
                {
                    s = s + f;
                }
                su = su + (((2.0f * kf) + 1.0f) * f * f);
                f1 = f0;
                f0 = f;
            }

            q = sqrtf(su);
            c = c * xa / q;
            s = s * xa / q;
        }
        //
        //  4.5 <= x
        //
        else
        {
            r = 1.0f;
            f = 1.0f;
            for (k = 1; k <= ITERATIONS_B; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.25f * r * ((4.0f * kf) - 1.0f) * ((4.0f * kf) - 3.0f) / t2;
                f = f + r;
            }
            r = 1.0f / (px * xa);
            g = r;
            for (k = 1; k <= ITERATIONS_C; k++)
            {
                const float kf = static_cast<float>(k);
                r = -0.25f * r * ((4.0f * kf) + 1.0f) * ((4.0f * kf) - 1.0f) / t2;
                g = g + r;
            }

            const float wrappedTurns = static_cast<float>(static_cast<int>(t / (2.0f * PI_F)));
            t0 = t - (wrappedTurns * 2.0f * PI_F);
            c = 0.5f + (f * sinf(t0) - g * cosf(t0)) / px;
            s = 0.5f - (f * cosf(t0) + g * sinf(t0)) / px;
        }
        //
        //  Apply symmetry for x < 0.
        //
        if (x < 0.0f)
        {
            c = -c;
            s = -s;
        }

        return;
    }
    //****************************************************************************80

    float fresnel_cosf(float x)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel_cos() evaluates the Fresnel cosine integral C(x).
        //
        //  Licensing:
        //
        //    This code is distributed under the MIT license.
        //
        //  Modified:
        //
        //    12 July 2025
        //
        //  Author:
        //
        //    John Burkardt.
        //
        //  Reference:
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    real X: the argument.
        //
        //  Output:
        //
        //    real C: the Fresnel cosine integral value at X.
        //
    {
        float c;
        float s;

        fresnelf(x, c, s);

        return c;
    }
    //****************************************************************************80

    float fresnel_sinf(float x)

        //****************************************************************************80
        //
        //  Purpose:
        //
        //    fresnel_sin() evaluates the Fresnel sine integral S(x).
        //
        //  Licensing:
        //
        //    This code is distributed under the MIT license.
        //
        //  Modified:
        //
        //    12 July 2025
        //
        //  Author:
        //
        //    John Burkardt.
        //
        //  Reference:
        //
        //    Shanjie Zhang, Jianming Jin,
        //    Computation of Special Functions,
        //    Wiley, 1996,
        //    ISBN: 0-471-11963-6,
        //    LC: QA351.C45.
        //
        //  Input:
        //
        //    real X: the argument.
        //
        //  Output:
        //
        //    real S: the Fresnel sine integral value at X.
        //
    {
        float c;
        float s;

        fresnelf(x, c, s);

        return s;
    }
}
