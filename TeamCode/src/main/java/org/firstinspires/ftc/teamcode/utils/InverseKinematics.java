package org.firstinspires.ftc.teamcode.utils;

public class InverseKinematics {
    public static double[] inverseKinematics(double length1, double length2, double x, double y, boolean isType1) {
        double c = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double p = Math.toDegrees(Math.atan2(y, x));

        if (c >= length1 + length2) {
            return new double[] {p, p};
        } else if (c <= Math.abs(length1-length2)) {
            if (length1 >= length2) {
                return new double[] {p, (180 + p) % 360};
            } else {
                return new double[] {(180 + p) % 360, p};
            }
        } else {
            if (isType1) {
                double a1 = Math.PI - Math.acos((Math.pow(length1, 2) + Math.pow(length2, 2) - Math.pow(c, 2)) / (2 * length1 * length2));

                double ad1 = Math.atan2(y, x) - Math.atan2(Math.sin(a1) * length2, length1 + Math.cos(a1) * length2);

                double ax1 = Math.cos(ad1) * length1;
                double ay1 = Math.sin(ad1) * length1;

                double bx1 = ax1 + Math.cos(a1 + ad1) * length2;
                double by1 = ay1 + Math.sin(a1 + ad1) * length2;

                return new double[]{Math.toDegrees(Math.atan2(ay1, ax1)), Math.toDegrees(Math.atan2(by1 - ay1, bx1 - ax1))};
            } else {
                double a2 = Math.PI + Math.acos((Math.pow(length1, 2) + Math.pow(length2, 2) - Math.pow(c, 2)) / (2 * length1 * length2));
                double ad2 = Math.atan2(y, x) - Math.atan2(Math.sin(a2) * length2, length1 + Math.cos(a2) * length2);

                double ax2 = Math.cos(ad2) * length1;
                double ay2 = Math.sin(ad2) * length1;

                double bx2 = ax2 + Math.cos(a2 + ad2) * length2;
                double by2 = ay2 + Math.sin(a2 + ad2) * length2;

                return new double[]{Math.toDegrees(Math.atan2(ay2, ax2)), Math.toDegrees(Math.atan2(by2 - ay2, bx2 - ax2))};
            }
        }
    }
}