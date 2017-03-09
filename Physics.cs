using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class Physics
    {
        const double SQRT3 = 1.7320508075688772935274463415059;
        const double wB = 300e-3; // actuator radius (m)
        const double uP = 60e-3;  // end effector radius (m)
        const double l = 1.0;    // upper-link length (m)
        const double L = 0.3;    // lower-link length (m)

        /// <summary>
        /// Forward kinematics for the delta robot
        /// </summary>
        /// <param name="th1">Theta 1 from the MCU</param>
        /// <param name="th2">Theta 2 from the MCU</param>
        /// <param name="th3">Theta 3 from the MCU</param>
        /// <returns>An array of cartesian positions relative to the robot's center</returns>
        public static double[] physics_fkin(double th1, double th2, double th3)
        {
            const double sP = 3 * uP / SQRT3;
            const double wP = uP / 2;

            double y1 = -wB - L * Math.Cos(th1) + uP;
            double z1 = L * Math.Sin(th1);

            double x2 = 0.5 * (SQRT3 * (wB + L * Math.Cos(th2)) - sP);
            double y2 = 0.5 * (wB + L * Math.Cos(th2)) - wP;
            double z2 = L * Math.Sin(th2);

            double x3 = 0.5 * (-SQRT3 * (wB + L * Math.Cos(th3)) + sP);
            double y3 = 0.5 * (wB + L * Math.Cos(th3)) - wP;
            double z3 = L * Math.Sin(th3);

            double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

            double w1 = y1 * y1 + z1 * z1;
            double w2 = x2 * x2 + y2 * y2 + z2 * z2;
            double w3 = x3 * x3 + y3 * y3 + z3 * z3;

            double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
            double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

            double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
            double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

            double a = a1 * a1 + a2 * a2 + dnm * dnm;
            double b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
            double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - l * l);

            double d = b * b - 4.0 * a * c;
            double[] p = new double[3];
            if (d >= 0)
            {
                p[2] = 0.5 * (b + Math.Sqrt(d)) / a;
                p[0] = (-a1 * p[2] + b1) / dnm;
                p[1] = (-a2 * p[2] + b2) / dnm;
                return p;
            }
            return null;
        }
    }
}
