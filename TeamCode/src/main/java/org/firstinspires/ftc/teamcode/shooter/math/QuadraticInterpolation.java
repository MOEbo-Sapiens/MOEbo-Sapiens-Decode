package org.firstinspires.ftc.teamcode.shooter.math;

import smile.interpolation.Interpolation;

public class QuadraticInterpolation implements Interpolation {
    private final double a, b, c;

    public QuadraticInterpolation(double[] x, double[] y) {
        if (x.length != y.length || x.length < 3) {
            throw new IllegalArgumentException("Need at least 3 points with matching x and y arrays");
        }

        // Least squares fit for y = ax² + bx + c
        int n = x.length;
        double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (int i = 0; i < n; i++) {
            double xi = x[i];
            double yi = y[i];
            double xi2 = xi * xi;

            sumX += xi;
            sumX2 += xi2;
            sumX3 += xi2 * xi;
            sumX4 += xi2 * xi2;
            sumY += yi;
            sumXY += xi * yi;
            sumX2Y += xi2 * yi;
        }

        // Solve 3x3 system using Cramer's rule
        // | n     sumX   sumX2  | | c |   | sumY   |
        // | sumX  sumX2  sumX3  | | b | = | sumXY  |
        // | sumX2 sumX3  sumX4  | | a |   | sumX2Y |

        double[][] mat = {
                {n,     sumX,  sumX2},
                {sumX,  sumX2, sumX3},
                {sumX2, sumX3, sumX4}
        };
        double[] rhs = {sumY, sumXY, sumX2Y};

        double det = determinant3x3(mat);

        double[][] matA = {
                {n,     sumX,  rhs[0]},
                {sumX,  sumX2, rhs[1]},
                {sumX2, sumX3, rhs[2]}
        };

        double[][] matB = {
                {n,     rhs[0], sumX2},
                {sumX,  rhs[1], sumX3},
                {sumX2, rhs[2], sumX4}
        };

        double[][] matC = {
                {rhs[0], sumX,  sumX2},
                {rhs[1], sumX2, sumX3},
                {rhs[2], sumX3, sumX4}
        };

        this.a = determinant3x3(matA) / det;
        this.b = determinant3x3(matB) / det;
        this.c = determinant3x3(matC) / det;
    }

    private static double determinant3x3(double[][] m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    @Override
    public double interpolate(double x) {
        return a * x * x + b * x + c;
    }

    public double getA() { return a; }
    public double getB() { return b; }
    public double getC() { return c; }
}