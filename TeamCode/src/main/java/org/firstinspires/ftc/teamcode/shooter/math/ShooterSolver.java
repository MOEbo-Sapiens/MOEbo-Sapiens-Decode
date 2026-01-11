package org.firstinspires.ftc.teamcode.shooter.math;

/**
 * ShooterSolver: Computes optimal hood angle, turret angle, and flywheel speed
 * for shooting while the robot is moving.
 * 
 * This solver handles the coupled system where:
 *   - Hood angle affects contact time
 *   - Contact time affects release pose (robot moves during shot)
 *   - Release pose affects distance to goal
 *   - Distance affects flywheel speed (via regression)
 *   - Flywheel speed affects contact time (circular dependency)
 *   - Robot velocity at release must be compensated by turret angle
 *   - Final ball velocity must satisfy trajectory equation
 * 
 * Solution approach:
 *   - Outer loop: Binary search on hood angle θ_h
 *   - Inner loop: Fixed-point iteration for (ω_f, t_contact, d)
 *   - Analytical solution for turret compensation
 *   - Trajectory equation determines if θ_h is correct
 */
public class ShooterSolver {

    // =========================================================================
    // PHYSICAL CONSTANTS
    // =========================================================================
    
    /** Gravity in inches/s² */
    private static final double g = 386.0885;
    
    /** Air density in slugs/in³ */
    private static final double rho = 1.376e-6;
    
    /** Drag coefficient (measured) */
    private static final double CD = 0.534;;
    
    /** Flywheel radius in inches */
    private static final double R_FLYWHEEL = 1.41732;
    
    /** Ball radius in inches */
    private static final double R_BALL = 2.5;
    
    /** Ball mass in slugs */
    private static final double MASS_BALL = 0.0748 * 0.0685218;
    
    /** Flywheel moment of inertia in slug·in² (MEASURE THIS) */
    private static final double J_FLYWHEEL = 0.01; // TODO: Measure actual value
    
    /** Drag constant k = ρ·CD·A / (2m) */
    private static final double k = rho * CD * Math.PI * R_BALL * R_BALL / (2 * MASS_BALL);
    
    // =========================================================================
    // SHOOTER GEOMETRY CONSTANTS
    // =========================================================================
    
    /** Angle at which ball first contacts flywheel (radians) */
    private static final double THETA_FIRST_CONTACT = Math.toRadians(-18.241);
    
    /** Minimum hood angle (radians) */
    private static final double MIN_HOOD_ANGLE = Math.toRadians(40);
    
    /** Maximum hood angle (radians) */
    private static final double MAX_HOOD_ANGLE = Math.toRadians(80);
    
    /** Arc angle at which slip ends (radians) - CALIBRATE THIS */
    private static final double THETA_SLIP = Math.toRadians(60); // TODO: Calibrate
    
    /** Fixed delay before ball contacts flywheel (seconds) */
    private static final double T_INIT = 0.05; // TODO: Measure actual value
    
    /** Height of shooter exit above ground (inches) */
    private static final double LAUNCH_HEIGHT = 12.0; // TODO: Measure actual value
    
    /** Height of goal (inches) */
    private static final double GOAL_HEIGHT = 24.0; // TODO: Set actual value
    
    // =========================================================================
    // DERIVED CONSTANTS
    // =========================================================================
    
    /** Combined radius for arc calculations */
    private static final double R_COMBINED = R_FLYWHEEL + R_BALL;
    
    /** Maximum velocity ratio A_max = 2J / (7J + 2mR²) */
    private static final double A_MAX = 2 * J_FLYWHEEL / 
            (7 * J_FLYWHEEL + 2 * MASS_BALL * R_FLYWHEEL * R_FLYWHEEL);
    
    // =========================================================================
    // SOLVER PARAMETERS
    // =========================================================================
    
    /** Tolerance for binary search on hood angle (radians) */
    private static final double HOOD_ANGLE_TOLERANCE = Math.toRadians(0.1);
    
    /** Tolerance for inner loop convergence (rad/s) */
    private static final double OMEGA_TOLERANCE = 0.01;
    
    /** Maximum iterations for inner loop */
    private static final int MAX_INNER_ITERATIONS = 10;
    
    /** Maximum iterations for outer loop (binary search) */
    private static final int MAX_OUTER_ITERATIONS = 20;
    
    // =========================================================================
    // FLYWHEEL SPEED REGRESSION COEFFICIENTS
    // =========================================================================
    
    /** Linear regression: ω_f = REGRESSION_SLOPE * distance + REGRESSION_INTERCEPT */
    private static final double REGRESSION_SLOPE = 1.0;     // TODO: Calibrate from data
    private static final double REGRESSION_INTERCEPT = 50.0; // TODO: Calibrate from data
    
    // =========================================================================
    // RESULT CLASS
    // =========================================================================
    
    /**
     * Container for solver results.
     */
    public static class ShotSolution {
        /** Hood angle in radians */
        public final double hoodAngle;
        
        /** Turret angle (robot-frame) in radians */
        public final double turretAngle;
        
        /** Flywheel angular velocity in rad/s */
        public final double flywheelSpeed;
        
        /** Total time from decision to release in seconds */
        public final double totalTime;
        
        /** Distance to goal at release in inches */
        public final double distanceToGoal;
        
        /** Whether a valid solution was found */
        public final boolean isValid;
        
        /** Error message if solution is invalid */
        public final String errorMessage;
        
        public ShotSolution(double hoodAngle, double turretAngle, double flywheelSpeed,
                           double totalTime, double distanceToGoal) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelSpeed = flywheelSpeed;
            this.totalTime = totalTime;
            this.distanceToGoal = distanceToGoal;
            this.isValid = true;
            this.errorMessage = null;
        }
        
        public ShotSolution(String errorMessage) {
            this.hoodAngle = 0;
            this.turretAngle = 0;
            this.flywheelSpeed = 0;
            this.totalTime = 0;
            this.distanceToGoal = 0;
            this.isValid = false;
            this.errorMessage = errorMessage;
        }
    }
    
    // =========================================================================
    // HELPER CLASSES FOR INTERMEDIATE VALUES
    // =========================================================================
    
    /**
     * Intermediate values computed by the inner loop.
     */
    private static class InnerLoopResult {
        double omegaFlywheel;
        double tContact;
        double tTotal;
        double distance;
        double releasePoseX;
        double releasePoseY;
        double releasePoseHeading;
        double robotVelX;  // Robot velocity at release (field frame)
        double robotVelY;
    }
    
    // =========================================================================
    // MAIN SOLVER METHOD
    // =========================================================================
    
    /**
     * Solves for the optimal shot parameters given robot state and goal position.
     * 
     * @param robotX Current robot X position (inches, field frame)
     * @param robotY Current robot Y position (inches, field frame)
     * @param robotHeading Current robot heading (radians, field frame)
     * @param robotVelX Robot velocity X component (inches/s, field frame)
     * @param robotVelY Robot velocity Y component (inches/s, field frame)
     * @param robotOmega Robot angular velocity (rad/s)
     * @param goalX Goal X position (inches, field frame)
     * @param goalY Goal Y position (inches, field frame)
     * @return ShotSolution containing hood angle, turret angle, flywheel speed, etc.
     */
    public static ShotSolution solve(
            double robotX, double robotY, double robotHeading,
            double robotVelX, double robotVelY, double robotOmega,
            double goalX, double goalY) {
        
        // ---------------------------------------------------------------------
        // OUTER LOOP: Binary search on hood angle
        // ---------------------------------------------------------------------
        
        double thetaLow = MIN_HOOD_ANGLE;
        double thetaHigh = MAX_HOOD_ANGLE;
        
        // Store best solution found
        double bestHoodAngle = (thetaLow + thetaHigh) / 2;
        InnerLoopResult bestInner = null;
        double bestTurretAngle = 0;
        double bestError = Double.MAX_VALUE;
        
        for (int outerIter = 0; outerIter < MAX_OUTER_ITERATIONS; outerIter++) {
            
            // Check convergence
            if (thetaHigh - thetaLow < HOOD_ANGLE_TOLERANCE) {
                break;
            }
            
            double thetaHood = (thetaLow + thetaHigh) / 2;
            
            // -----------------------------------------------------------------
            // INNER LOOP: Solve for (ω_f, t_contact, d) at this hood angle
            // -----------------------------------------------------------------
            
            InnerLoopResult inner = solveInnerLoop(
                    thetaHood,
                    robotX, robotY, robotHeading,
                    robotVelX, robotVelY, robotOmega,
                    goalX, goalY);
            
            if (inner == null) {
                // Inner loop failed to converge; try different hood angle
                thetaLow = thetaHood;
                continue;
            }
            
            // -----------------------------------------------------------------
            // TURRET COMPENSATION: Solve for φ' analytically
            // -----------------------------------------------------------------
            
            TurretSolution turret = solveTurretAngle(
                    thetaHood,
                    inner.omegaFlywheel,
                    inner.releasePoseX, inner.releasePoseY, inner.releasePoseHeading,
                    inner.robotVelX, inner.robotVelY,
                    goalX, goalY);
            
            if (!turret.isValid) {
                // Robot moving too fast to compensate; widen search
                thetaLow = thetaHood;
                continue;
            }
            
            // -----------------------------------------------------------------
            // TRAJECTORY CHECK: Compute vertical error
            // -----------------------------------------------------------------
            
            double error = computeTrajectoryError(
                    thetaHood,
                    inner.omegaFlywheel,
                    inner.distance,
                    turret.horizontalSpeed);
            
            // Update best solution
            if (Math.abs(error) < Math.abs(bestError)) {
                bestError = error;
                bestHoodAngle = thetaHood;
                bestInner = inner;
                bestTurretAngle = turret.turretAngle;
            }
            
            // -----------------------------------------------------------------
            // BINARY SEARCH UPDATE
            // -----------------------------------------------------------------
            
            if (error > 0) {
                // Ball going too high → decrease hood angle
                thetaHigh = thetaHood;
            } else {
                // Ball going too low → increase hood angle
                thetaLow = thetaHood;
            }
        }
        
        // ---------------------------------------------------------------------
        // RETURN RESULT
        // ---------------------------------------------------------------------
        
        if (bestInner == null) {
            return new ShotSolution("Failed to find valid solution");
        }
        
        // Check if error is acceptable (within ~1 inch of goal height)
        if (Math.abs(bestError) > 1.0) {
            return new ShotSolution("Solution error too large: " + bestError + " inches");
        }
        
        return new ShotSolution(
                bestHoodAngle,
                bestTurretAngle,
                bestInner.omegaFlywheel,
                bestInner.tTotal,
                bestInner.distance);
    }
    
    // =========================================================================
    // INNER LOOP: Fixed-point iteration for (ω_f, t_contact, d)
    // =========================================================================
    
    /**
     * Solves for flywheel speed, contact time, and distance via fixed-point iteration.
     * 
     * @return InnerLoopResult or null if failed to converge
     */
    private static InnerLoopResult solveInnerLoop(
            double thetaHood,
            double robotX, double robotY, double robotHeading,
            double robotVelX, double robotVelY, double robotOmega,
            double goalX, double goalY) {
        
        // Initial guess for flywheel speed based on current distance
        double dxInitial = goalX - robotX;
        double dyInitial = goalY - robotY;
        double distInitial = Math.sqrt(dxInitial * dxInitial + dyInitial * dyInitial);
        double omegaF = flywheelSpeedFromDistance(distInitial);
        
        InnerLoopResult result = new InnerLoopResult();
        
        for (int i = 0; i < MAX_INNER_ITERATIONS; i++) {
            
            // Compute contact time from hood angle and flywheel speed
            double tContact = computeContactTime(thetaHood, omegaF);
            double tTotal = T_INIT + tContact;
            
            // Compute release pose using twist integration
            double[] releasePose = twistIntegrate(
                    robotX, robotY, robotHeading,
                    robotVelX, robotVelY, robotOmega,
                    tTotal);
            
            // Compute distance to goal at release
            double dx = goalX - releasePose[0];
            double dy = goalY - releasePose[1];
            double dist = Math.sqrt(dx * dx + dy * dy);
            
            // Compute new flywheel speed from regression
            double omegaFNew = flywheelSpeedFromDistance(dist);
            
            // Check convergence
            if (Math.abs(omegaFNew - omegaF) < OMEGA_TOLERANCE) {
                // Converged! Store results
                result.omegaFlywheel = omegaFNew;
                result.tContact = tContact;
                result.tTotal = tTotal;
                result.distance = dist;
                result.releasePoseX = releasePose[0];
                result.releasePoseY = releasePose[1];
                result.releasePoseHeading = releasePose[2];
                
                // Compute robot velocity at release (rotated by ω * t_total)
                double dtheta = robotOmega * tTotal;
                double cos = Math.cos(dtheta);
                double sin = Math.sin(dtheta);
                result.robotVelX = robotVelX * cos - robotVelY * sin;
                result.robotVelY = robotVelX * sin + robotVelY * cos;
                
                return result;
            }
            
            omegaF = omegaFNew;
        }
        
        // Failed to converge
        return null;
    }
    
    // =========================================================================
    // TURRET COMPENSATION: Analytical solution
    // =========================================================================
    
    /**
     * Result of turret angle calculation.
     */
    private static class TurretSolution {
        double turretAngle;      // Robot-frame turret angle (radians)
        double horizontalSpeed;  // Resultant horizontal ball speed toward goal (λ)
        boolean isValid;
    }
    
    /**
     * Solves for turret angle such that ball velocity points toward goal
     * after adding robot velocity.
     * 
     * The problem: find shooter direction β such that
     *   v_h · (cos β, sin β) + v_robot  points toward goal
     * 
     * where v_h = horizontal component of shooter velocity.
     * 
     * This is solved via the vector triangle / law of cosines.
     */
    private static TurretSolution solveTurretAngle(
            double thetaHood,
            double omegaFlywheel,
            double releaseX, double releaseY, double releaseHeading,
            double robotVelX, double robotVelY,
            double goalX, double goalY) {
        
        TurretSolution result = new TurretSolution();
        
        // Compute shooter velocity magnitude
        double vShooter = computeExitVelocity(thetaHood, omegaFlywheel);
        double vHorizontal = vShooter * Math.cos(thetaHood);  // Horizontal component
        
        // Direction to goal (α)
        double dx = goalX - releaseX;
        double dy = goalY - releaseY;
        double alpha = Math.atan2(dy, dx);
        
        // Unit vector toward goal
        double uX = Math.cos(alpha);
        double uY = Math.sin(alpha);
        
        // Project robot velocity onto goal direction
        double c = robotVelX * uX + robotVelY * uY;
        
        // Robot velocity perpendicular to goal direction (squared)
        double vRobotSq = robotVelX * robotVelX + robotVelY * robotVelY;
        double vPerpSq = vRobotSq - c * c;
        
        // Check if solution exists: need v_h >= |v_perp|
        if (vHorizontal * vHorizontal < vPerpSq) {
            result.isValid = false;
            return result;
        }
        
        // Solve for λ (resultant horizontal speed toward goal)
        // From: λ² - 2cλ + (|v_robot|² - v_h²) = 0
        // λ = c + sqrt(v_h² - v_perp²)
        double lambda = c + Math.sqrt(vHorizontal * vHorizontal - vPerpSq);
        
        // Compute shooter direction β (field frame)
        // v_shooter = λ·û - v_robot
        double vShooterX = lambda * uX - robotVelX;
        double vShooterY = lambda * uY - robotVelY;
        double beta = Math.atan2(vShooterY, vShooterX);
        
        // Convert to turret angle (robot frame)
        double turretAngle = wrapAngle(beta - releaseHeading);
        
        result.turretAngle = turretAngle;
        result.horizontalSpeed = lambda;
        result.isValid = true;
        
        return result;
    }
    
    // =========================================================================
    // TRAJECTORY ERROR CALCULATION
    // =========================================================================
    
    /**
     * Computes the vertical error: how much the ball overshoots (+) or
     * undershoots (-) the goal height.
     * 
     * @param thetaHood Hood angle (determines vertical velocity component)
     * @param omegaFlywheel Flywheel speed (affects ball speed)
     * @param distance Horizontal distance to goal
     * @param lambdaHorizontal Horizontal speed toward goal (from turret solve)
     * @return Vertical error in inches (positive = too high)
     */
    private static double computeTrajectoryError(
            double thetaHood,
            double omegaFlywheel,
            double distance,
            double lambdaHorizontal) {
        
        // Vertical component of ball velocity
        double vShooter = computeExitVelocity(thetaHood, omegaFlywheel);
        double vVertical = vShooter * Math.sin(thetaHood);
        
        // Time of flight (with horizontal drag)
        // x = (1/k) * ln(1 + k * v_h * t)
        // Solving for t: t = (exp(k * x) - 1) / (k * v_h)
        double E = Math.exp(k * distance) - 1;
        double timeOfFlight = E / (k * lambdaHorizontal);
        
        // Vertical displacement (simplified drag model)
        // y = v_v * t - 0.5 * g * t²
        double deltaY = vVertical * timeOfFlight - 0.5 * g * timeOfFlight * timeOfFlight;
        
        // Target vertical displacement
        double targetDeltaY = GOAL_HEIGHT - LAUNCH_HEIGHT;
        
        // Error (positive means ball goes too high)
        return deltaY - targetDeltaY;
    }
    
    // =========================================================================
    // PHYSICS HELPER METHODS
    // =========================================================================
    
    /**
     * Computes the velocity ratio A(θ) as function of hood angle.
     * 
     * A(θ) = A_max * min(1, sqrt((θ - θ₀) / θ_slip))
     */
    private static double computeVelocityRatio(double thetaHood) {
        double thetaArc = thetaHood - THETA_FIRST_CONTACT;
        
        if (thetaArc <= 0) {
            return 0;
        }
        
        if (thetaArc >= THETA_SLIP) {
            return A_MAX;
        }
        
        return A_MAX * Math.sqrt(thetaArc / THETA_SLIP);
    }
    
    /**
     * Computes ball exit velocity given hood angle and flywheel speed.
     * 
     * v_ball = A(θ) * ω_flywheel * R_flywheel
     */
    private static double computeExitVelocity(double thetaHood, double omegaFlywheel) {
        double A = computeVelocityRatio(thetaHood);
        return A * omegaFlywheel * R_FLYWHEEL;
    }
    
    /**
     * Computes contact time as function of hood angle and flywheel speed.
     * 
     * Slipping (θ_arc < θ_slip):
     *   t = 2(R+r) * sqrt(θ_arc * θ_slip) / v_max
     * 
     * Saturated (θ_arc >= θ_slip):
     *   t = (R+r)(θ_arc + θ_slip) / v_max
     */
    private static double computeContactTime(double thetaHood, double omegaFlywheel) {
        double thetaArc = thetaHood - THETA_FIRST_CONTACT;
        
        if (thetaArc <= 0) {
            return 0;
        }
        
        // Maximum velocity at this flywheel speed
        double vMax = A_MAX * omegaFlywheel * R_FLYWHEEL;
        
        if (vMax <= 0) {
            return Double.MAX_VALUE;  // Flywheel not spinning
        }
        
        if (thetaArc < THETA_SLIP) {
            // Slipping: t = 2(R+r) * sqrt(θ_arc * θ_slip) / v_max
            return 2 * R_COMBINED * Math.sqrt(thetaArc * THETA_SLIP) / vMax;
        } else {
            // Saturated: t = (R+r)(θ_arc + θ_slip) / v_max
            return R_COMBINED * (thetaArc + THETA_SLIP) / vMax;
        }
    }
    
    /**
     * Computes flywheel speed from distance using linear regression.
     * 
     * ω_f = slope * d + intercept
     */
    private static double flywheelSpeedFromDistance(double distance) {
        return REGRESSION_SLOPE * distance + REGRESSION_INTERCEPT;
    }
    
    // =========================================================================
    // TWIST INTEGRATION
    // =========================================================================
    
    /**
     * Integrates robot motion with constant body-frame velocity and angular velocity.
     * Uses exact exponential map (SE(2) integration).
     * 
     * @param x0 Initial X position (field frame)
     * @param y0 Initial Y position (field frame)
     * @param theta0 Initial heading (field frame)
     * @param vxField Velocity X (field frame)
     * @param vyField Velocity Y (field frame)
     * @param omega Angular velocity
     * @param dt Time step
     * @return [x_final, y_final, theta_final]
     */
    private static double[] twistIntegrate(
            double x0, double y0, double theta0,
            double vxField, double vyField, double omega,
            double dt) {
        
        double dtheta = omega * dt;
        double newHeading = wrapAngle(theta0 + dtheta);
        
        // Convert field velocity to body frame
        double cosInit = Math.cos(theta0);
        double sinInit = Math.sin(theta0);
        double vxBody =  vxField * cosInit + vyField * sinInit;
        double vyBody = -vxField * sinInit + vyField * cosInit;
        
        double dxBody, dyBody;
        
        if (Math.abs(omega) < 1e-9) {
            // Straight line case (avoid divide by zero)
            dxBody = vxBody * dt;
            dyBody = vyBody * dt;
        } else {
            // Arc case: use exponential map
            double sinDtheta = Math.sin(dtheta);
            double cosDtheta = Math.cos(dtheta);
            dxBody = (vxBody * sinDtheta - vyBody * (1 - cosDtheta)) / omega;
            dyBody = (vxBody * (1 - cosDtheta) + vyBody * sinDtheta) / omega;
        }
        
        // Rotate displacement back to field frame
        double dxField = dxBody * cosInit - dyBody * sinInit;
        double dyField = dxBody * sinInit + dyBody * cosInit;
        
        return new double[] {
            x0 + dxField,
            y0 + dyField,
            newHeading
        };
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    /**
     * Wraps angle to [-π, π].
     */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
