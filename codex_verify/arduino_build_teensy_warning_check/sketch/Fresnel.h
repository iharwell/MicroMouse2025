#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Fresnel.h"
#pragma once
#include "Defines.h"
//#define FRESNEL_PRECISION 4
#define FRESNEL_PRECISION 5
namespace MazeMap
{

	constexpr float FIRST_BOUNDARY = 1.8f;
	constexpr float SECOND_BOUNDARY = 3.5f;
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

	EXPORT void fresnel(double x, double& c, double& s);
	EXPORT double fresnel_cos(double x);
	EXPORT double fresnel_sin(double x);

	EXPORT void fresnelf(float x, float& c, float& s);
	EXPORT void fresnelf_test(float x, float& c, float& s, float initial_f0);
	EXPORT float fresnel_cosf(float x);
	EXPORT float fresnel_sinf(float x);
}