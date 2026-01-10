package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Vector2D;

public class RevAmpedProjectileMathWithDrag {
    private static final double g = 386.0885; // gravity in in/s^2

    // FIXED: Air density in slugs/in³ (0.0023769 slugs/ft³ ÷ 1728)
    private static final double rho = 1.376e-6; // slugs/in³

    // FIXED: Measured drag coefficient for FTC DECODE artifact
    private static final double CD = 0.534;

    // FIXED: Radius is 2.5 inches (diameter is 5 inches)
    private static final double radius = 2.5; // ball radius in inches

    private static final double A = Math.PI * radius * radius; // cross-sectional area in in²
                                                               // (≈19.63)

    // FIXED: Official mass is 0.165 lbs = 0.0748 kg
    private static final double massBall = 0.0748 * 0.0685218; // ball mass in slugs (≈0.00513)

    private static final double k = rho * CD * A / (2 * massBall); // Units: 1/in (≈0.0012)

    /**
     * Computes the coordinates of the ball after being launched with approximate quadratic drag
     * 
     * @param time seconds following launch
     * @param v_0 initial launch velocity in in/s
     * @param theta_0 initial launch angle in degrees
     * @param h_0 initial height in inches
     * @return coordinates (in inches)
     */
    public static Vector2D computePosition(double time, double v_0, double theta_0, double h_0) {
        double convertedTheta = Math.toRadians(theta_0);

        // Horizontal motion with quadratic drag
        double v0x = v_0 * Math.cos(convertedTheta);
        double x = (1.0 / k) * Math.log(1 + k * v0x * time);

        // Vertical motion with approximate quadratic drag
        double vy0 = v_0 * Math.sin(convertedTheta);
        // FIXED: Changed + to - for gravity term (gravity pulls DOWN)
        double y = h_0 + (vy0 / k) * Math.log(1 + k * vy0 * time) - 0.5 * g * time * time;

        return new Vector2D(x, y);
    }

    /**
     * Computes the launch angle θ (in radians) required for a projectile with approximate quadratic
     * drag to pass through a specified control point.
     *
     * @param distances the planar distance (x-coordinate) and height distance (y-coordinate) from
     *        launch to target
     * @param v0 the initial launch speed in inches per second
     * @param currentHoodRad fallback angle if no solution exists
     * @return the launch angle θ in radians required to reach the control point, or currentHoodRad
     *         if the speed v0 is too low to reach the point
     */
    public static double solveTheta(Vector2D distances, double v0, double currentHoodRad) {
        double dx = distances.getX();
        double dy = distances.getY();

        double E = Math.exp(k * dx) - 1;

        double a = g * E * E;
        double b = -2 * k * E * v0 * v0;
        double c = g * E * E + 2 * k * k * v0 * v0 * dy;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return currentHoodRad;
        }

        double T1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double T2 = (-b - Math.sqrt(discriminant)) / (2 * a);

        double theta1 = Math.atan(T1);
        double theta2 = Math.atan(T2);

        double theta = Math.min(theta1, theta2);

        if (theta > Math.PI / 2)
            theta = Math.PI / 2;

        return theta;
    }

    /**
     * Computes velocity components at time t
     * 
     * @param t time in seconds
     * @param v_0 initial velocity in in/s
     * @param theta_0 initial angle in radians
     * @return velocity vector (in/s)
     */
    public static Vector2D getVelocity(double t, double v_0, double theta_0) {
        double v_ox = v_0 * Math.cos(theta_0);
        double v_x = v_ox / (1 + k * v_ox * t);

        double v_oy = v_0 * Math.sin(theta_0);
        // Note: This is an approximation - vertical drag is more complex
        double v_y = v_oy / (1 + k * v_oy * t) - g * t;

        return new Vector2D(v_x, v_y);
    }

    /**
     * Confidence metric based on entry angle
     */
    public static double getConfidence(Vector2D distances, double v_0, double theta_0) {
        double t = invertX(distances.getX(), v_0, theta_0);
        Vector2D velocity = getVelocity(t, v_0, theta_0);
        double theta = velocity.getTheta();
        double invertedCost = Math.cos(Math.toRadians(3) - theta);
        return Range.scale(invertedCost, Math.cos(Math.toRadians(8)), 1.0, 0.25, 0.95);
    }

    /**
     * Computes time to reach horizontal position xPos
     */
    public static double invertX(double xPos, double v_0, double theta_0) {
        double v_0x = v_0 * Math.cos(theta_0);
        double numerator = Math.exp(k * xPos) - 1;
        double denominator = k * v_0x;
        return numerator / denominator;
    }
}
