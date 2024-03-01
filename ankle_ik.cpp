#include "humanoid_control/kinematic/ankle_ik.h"

#include <cassert>
#include <cmath>

namespace pi
{
    std::array<double, 2> ankle_ik(double d, double L1, double h1, double h2, double tx, double ty)
    {
        double cx = cos(tx);
        double sx = sin(tx);
        double cy = cos(ty);
        double sy = sin(ty);

        double AL = - L1 * L1 * cy + L1 * d * sx * sy;
        double BL = - L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
        double CL = -(L1 * L1 + d * d - d * d *cx - L1 * h1 * sy - d * h1 * sx * cy);

        double LenL = sqrt(AL * AL + BL * BL);

        double AR = - L1 * L1 * cy - L1 * d * sx * sy;
        double BR = - L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
        double CR = -(L1 * L1 + d * d - d * d *cx - L1 * h2 * sy + d * h2 * sx * cy);

        double LenR = sqrt(AR * AR + BR * BR);

        if (LenL <= abs(CL) || LenR <= abs(CR))
        {
            return {0.0, 0.0};
        }
        else
        {
            double tL_1 = asin(CL / LenL) - asin(AL / LenL);
            double tL_2 = asin(CL / LenL) + acos(BL / LenL);

            double tR_1 = asin(CR / LenR) - asin(AR / LenR);
            double tR_2 = asin(CR / LenR) + acos(BR / LenR);

            assert(fabs(tL_1 - tL_2) < 1e-3 && "tL_1 - tL_2 > 1e-3");
            assert(fabs(tR_1 - tR_2) < 1e-3 && "tR_1 - tR_2 > 1e-3");

            return {tL_1, tR_1};
        }
    }
} // namespace pi
