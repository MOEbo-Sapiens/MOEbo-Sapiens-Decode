package org.firstinspires.ftc.teamcode.shooter;

/**
 * ShooterSolver: Computes optimal hood angle, turret angle, and flywheel speed for shooting while
 * the robot is moving.
 * 
 * SOLVER APPROACH: ================ Rather than binary search (which assumes monotonicity), we use
 * Newton-Raphson on the "closest approach" residual:
 * 
 * 1. For a given θ_h, compute the full ball trajectory 2. Find the point on the trajectory closest
 * to the goal 3. r(θ_h) = signed distance from trajectory to goal at closest approach 4. Compute
 * r'(θ_h) numerically 5. Newton update: θ_h ← θ_h - r / r' 6. Converged when |r| < tolerance
 * 
 * This handles non-monotonic relationships between θ_h and trajectory.
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
    private static final double CD = 0.44;

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

    /**
     * Launch angle offset: launch_angle = LAUNCH_ANGLE_OFFSET - hood_angle
     * 
     * From calibration: hood 40° → launch 50° (hood min → launch max) hood 80° → launch 10° (hood
     * max → launch min)
     * 
     * So: launch = 90° - hood
     */
    private static final double LAUNCH_ANGLE_OFFSET = Math.toRadians(90);

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
    private static final double A_MAX =
            2 * J_FLYWHEEL / (7 * J_FLYWHEEL + 2 * MASS_BALL * R_FLYWHEEL * R_FLYWHEEL);

    // =========================================================================
    // SOLVER PARAMETERS
    // =========================================================================

    /** Tolerance for Newton-Raphson convergence (inches) */
    private static final double RESIDUAL_TOLERANCE = 0.5;

    /** Step size for numerical derivative */
    private static final double DERIVATIVE_DELTA = Math.toRadians(0.5);

    /** Tolerance for inner loop convergence (rad/s) */
    private static final double OMEGA_TOLERANCE = 0.01;

    /** Maximum iterations for inner loop */
    private static final int MAX_INNER_ITERATIONS = 10;

    /** Maximum iterations for Newton-Raphson */
    private static final int MAX_NEWTON_ITERATIONS = 15;

    /** Time step for trajectory sampling (seconds) */
    private static final double TRAJECTORY_DT = 0.005;

    /** Maximum trajectory time to search (seconds) */
    private static final double MAX_TRAJECTORY_TIME = 3.0;

    // =========================================================================
    // FLYWHEEL SPEED REGRESSION COEFFICIENTS
    // =========================================================================

    /** Linear regression: ω_f = REGRESSION_SLOPE * distance + REGRESSION_INTERCEPT */
    private static final double REGRESSION_SLOPE = 1.0; // TODO: Calibrate
    private static final double REGRESSION_INTERCEPT = 50.0; // TODO: Calibrate

    // =========================================================================
    // RESULT CLASSES
    // =========================================================================

    /**
     * Container for solver results.
     */
    public static class ShotSolution {
        public final double hoodAngle;
        public final double turretAngle;
        public final double flywheelSpeed;
        public final double totalTime;
        public final double distanceToGoal;
        public final double residual; // Distance from trajectory to goal at closest point
        public final boolean isValid;
        public final String errorMessage;

        public ShotSolution(double hoodAngle, double turretAngle, double flywheelSpeed,
                double totalTime, double distanceToGoal, double residual) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelSpeed = flywheelSpeed;
            this.totalTime = totalTime;
            this.distanceToGoal = distanceToGoal;
            this.residual = residual;
            this.isValid = true;
            this.errorMessage = null;
        }

        public ShotSolution(String errorMessage) {
            this.hoodAngle = 0;
            this.turretAngle = 0;
            this.flywheelSpeed = 0;
            this.totalTime = 0;
            this.distanceToGoal = 0;
            this.residual = Double.MAX_VALUE;
            this.isValid = false;
            this.errorMessage = errorMessage;
        }
    }

    /**
     * Intermediate state computed for a given hood angle.
     */
    private static class ShotState {
        // From inner loop
        double omegaFlywheel;
        double tContact;
        double tTotal;
        double distance;
        double releasePoseX;
        double releasePoseY;
        double releasePoseHeading;
        double robotVelX;
        double robotVelY;

        // From turret solve
        double turretAngle;
        double horizontalSpeed; // λ - horizontal ball speed toward goal
        double verticalSpeed; // Vertical ball speed
        double launchDirectionX; // Unit vector of ball horizontal direction
        double launchDirectionY;
        boolean turretValid;

        // From trajectory analysis
        double closestT; // Time of closest approach
        double closestX; // X position at closest approach
        double closestY; // Y position at closest approach (horizontal plane)
        double closestZ; // Z position at closest approach (height)
        double residual; // Signed distance: negative = below goal, positive = above
    }

    // =========================================================================
    // MAIN SOLVER METHOD
    // =========================================================================

    /**
     * Solves for optimal shot parameters using Newton-Raphson on closest-approach residual.
     */
    public static ShotSolution solve(double robotX, double robotY, double robotHeading,
            double robotVelX, double robotVelY, double robotOmega, double goalX, double goalY) {

        // ---------------------------------------------------------------------
        // INITIAL GUESS: Start at middle of hood angle range
        // ---------------------------------------------------------------------

        double thetaH = (MIN_HOOD_ANGLE + MAX_HOOD_ANGLE) / 2;

        ShotState bestState = null;
        double bestResidual = Double.MAX_VALUE;
        double bestThetaH = thetaH;

        // ---------------------------------------------------------------------
        // NEWTON-RAPHSON ITERATION
        // ---------------------------------------------------------------------

        for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {

            // Compute state and residual at current θ_h
            ShotState state = computeFullState(thetaH, robotX, robotY, robotHeading, robotVelX,
                    robotVelY, robotOmega, goalX, goalY);

            if (state == null || !state.turretValid) {
                // Invalid state - try perturbing θ_h
                thetaH += Math.toRadians(5);
                if (thetaH > MAX_HOOD_ANGLE) {
                    thetaH = MIN_HOOD_ANGLE + Math.toRadians(5);
                }
                continue;
            }

            double r = state.residual;

            // Track best solution found
            if (Math.abs(r) < Math.abs(bestResidual)) {
                bestResidual = r;
                bestState = state;
                bestThetaH = thetaH;
            }

            // Check convergence
            if (Math.abs(r) < RESIDUAL_TOLERANCE) {
                return new ShotSolution(thetaH, state.turretAngle, state.omegaFlywheel,
                        state.tTotal, state.distance, r);
            }

            // -----------------------------------------------------------------
            // COMPUTE NUMERICAL DERIVATIVE dr/dθ_h
            // -----------------------------------------------------------------

            double thetaPlus = Math.min(thetaH + DERIVATIVE_DELTA, MAX_HOOD_ANGLE);
            double thetaMinus = Math.max(thetaH - DERIVATIVE_DELTA, MIN_HOOD_ANGLE);

            ShotState statePlus = computeFullState(thetaPlus, robotX, robotY, robotHeading,
                    robotVelX, robotVelY, robotOmega, goalX, goalY);

            ShotState stateMinus = computeFullState(thetaMinus, robotX, robotY, robotHeading,
                    robotVelX, robotVelY, robotOmega, goalX, goalY);

            // Handle edge cases where derivative computation fails
            if (statePlus == null || !statePlus.turretValid || stateMinus == null
                    || !stateMinus.turretValid) {
                // Fall back to gradient descent direction
                // Since dr/dθ < 0 (higher hood → lower launch → lower residual),
                // if r > 0 (too high), we need to INCREASE hood angle
                thetaH += Math.signum(r) * Math.toRadians(2);
                thetaH = clamp(thetaH, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
                continue;
            }

            double rPlus = statePlus.residual;
            double rMinus = stateMinus.residual;
            double dr_dtheta = (rPlus - rMinus) / (thetaPlus - thetaMinus);

            // -----------------------------------------------------------------
            // NEWTON UPDATE: θ_h ← θ_h - r / r'
            // -----------------------------------------------------------------

            if (Math.abs(dr_dtheta) < 1e-6) {
                // Derivative too small - use gradient descent instead
                // Since dr/dθ < 0, increase θ when r > 0
                thetaH += Math.signum(r) * Math.toRadians(2);
            } else {
                double delta = r / dr_dtheta;

                // Limit step size to prevent wild jumps
                double maxStep = Math.toRadians(10);
                delta = clamp(delta, -maxStep, maxStep);

                thetaH -= delta;
            }

            // Clamp to valid range
            thetaH = clamp(thetaH, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
        }

        // ---------------------------------------------------------------------
        // RETURN BEST SOLUTION FOUND
        // ---------------------------------------------------------------------

        if (bestState != null && Math.abs(bestResidual) < 5.0) {
            // Accept if within 5 inches - not perfect but usable
            return new ShotSolution(bestThetaH, bestState.turretAngle, bestState.omegaFlywheel,
                    bestState.tTotal, bestState.distance, bestResidual);
        }

        return new ShotSolution("Failed to converge. Best residual: " + bestResidual);
    }

    // =========================================================================
    // COMPUTE FULL STATE FOR A GIVEN HOOD ANGLE
    // =========================================================================

    /**
     * Computes all intermediate values and the residual for a given hood angle.
     */
    private static ShotState computeFullState(double thetaH, double robotX, double robotY,
            double robotHeading, double robotVelX, double robotVelY, double robotOmega,
            double goalX, double goalY) {

        ShotState state = new ShotState();

        // ---------------------------------------------------------------------
        // INNER LOOP: Solve for (ω_f, t_contact, d)
        // ---------------------------------------------------------------------

        if (!solveInnerLoop(state, thetaH, robotX, robotY, robotHeading, robotVelX, robotVelY,
                robotOmega, goalX, goalY)) {
            return null;
        }

        // ---------------------------------------------------------------------
        // TURRET COMPENSATION
        // ---------------------------------------------------------------------

        if (!solveTurretAngle(state, thetaH, goalX, goalY)) {
            return null;
        }

        // ---------------------------------------------------------------------
        // TRAJECTORY ANALYSIS: Find closest approach to goal
        // ---------------------------------------------------------------------

        computeClosestApproach(state, thetaH, goalX, goalY);

        return state;
    }

    // =========================================================================
    // INNER LOOP: Fixed-point iteration for (ω_f, t_contact, d)
    // =========================================================================

    /**
     * Solves for flywheel speed, contact time, and distance via fixed-point iteration. Results are
     * stored in the state object.
     * 
     * @return true if converged, false otherwise
     */
    private static boolean solveInnerLoop(ShotState state, double thetaH, double robotX,
            double robotY, double robotHeading, double robotVelX, double robotVelY,
            double robotOmega, double goalX, double goalY) {

        // Initial guess based on current distance
        double dxInitial = goalX - robotX;
        double dyInitial = goalY - robotY;
        double distInitial = Math.sqrt(dxInitial * dxInitial + dyInitial * dyInitial);
        double omegaF = flywheelSpeedFromDistance(distInitial);

        for (int i = 0; i < MAX_INNER_ITERATIONS; i++) {

            // Compute contact time
            double tContact = computeContactTime(thetaH, omegaF);
            double tTotal = T_INIT + tContact;

            // Compute release pose
            double[] releasePose = twistIntegrate(robotX, robotY, robotHeading, robotVelX,
                    robotVelY, robotOmega, tTotal);

            // Compute distance to goal
            double dx = goalX - releasePose[0];
            double dy = goalY - releasePose[1];
            double dist = Math.sqrt(dx * dx + dy * dy);

            // New flywheel speed
            double omegaFNew = flywheelSpeedFromDistance(dist);

            // Check convergence
            if (Math.abs(omegaFNew - omegaF) < OMEGA_TOLERANCE) {
                // Store results
                state.omegaFlywheel = omegaFNew;
                state.tContact = tContact;
                state.tTotal = tTotal;
                state.distance = dist;
                state.releasePoseX = releasePose[0];
                state.releasePoseY = releasePose[1];
                state.releasePoseHeading = releasePose[2];

                // Robot velocity at release
                double dtheta = robotOmega * tTotal;
                double cos = Math.cos(dtheta);
                double sin = Math.sin(dtheta);
                state.robotVelX = robotVelX * cos - robotVelY * sin;
                state.robotVelY = robotVelX * sin + robotVelY * cos;

                return true;
            }

            omegaF = omegaFNew;
        }

        return false; // Failed to converge
    }

    // =========================================================================
    // TURRET COMPENSATION
    // =========================================================================

    /**
     * Solves for turret angle using vector triangle method. Results stored in state object.
     * 
     * @return true if valid solution exists, false if robot too fast
     */
    private static boolean solveTurretAngle(ShotState state, double thetaH, double goalX,
            double goalY) {

        // Convert hood angle to launch angle for trajectory calculations
        double launchAngle = hoodAngleToLaunchAngle(thetaH);

        // Shooter velocity magnitude (depends on hood angle for contact arc)
        double vShooter = computeExitVelocity(thetaH, state.omegaFlywheel);

        // Velocity components use LAUNCH angle (actual trajectory direction)
        double vHorizontal = vShooter * Math.cos(launchAngle);
        double vVertical = vShooter * Math.sin(launchAngle);

        state.verticalSpeed = vVertical;

        // Direction to goal
        double dx = goalX - state.releasePoseX;
        double dy = goalY - state.releasePoseY;
        double distHoriz = Math.sqrt(dx * dx + dy * dy);

        if (distHoriz < 1e-6) {
            state.turretValid = false;
            return false;
        }

        double alpha = Math.atan2(dy, dx);
        double uX = Math.cos(alpha);
        double uY = Math.sin(alpha);

        // Project robot velocity onto goal direction
        double c = state.robotVelX * uX + state.robotVelY * uY;

        // Perpendicular component squared
        double vRobotSq = state.robotVelX * state.robotVelX + state.robotVelY * state.robotVelY;
        double vPerpSq = vRobotSq - c * c;

        // Check solvability
        if (vHorizontal * vHorizontal < vPerpSq) {
            state.turretValid = false;
            return false;
        }

        // Solve for λ (resultant speed toward goal)
        double lambda = c + Math.sqrt(vHorizontal * vHorizontal - vPerpSq);

        // Shooter direction (field frame)
        double vShooterX = lambda * uX - state.robotVelX;
        double vShooterY = lambda * uY - state.robotVelY;
        double beta = Math.atan2(vShooterY, vShooterX);

        // Store results
        state.turretAngle = wrapAngle(beta - state.releasePoseHeading);
        state.horizontalSpeed = lambda;
        state.launchDirectionX = uX;
        state.launchDirectionY = uY;
        state.turretValid = true;

        return true;
    }

    // =========================================================================
    // TRAJECTORY ANALYSIS: Find closest approach to goal
    // =========================================================================

    /**
     * Computes the trajectory and finds the point closest to the goal.
     * 
     * The trajectory is parameterized by time t: x(t) = release_x + (1/k) * ln(1 + k * λ * t) *
     * dir_x y(t) = release_y + (1/k) * ln(1 + k * λ * t) * dir_y z(t) = launch_height + v_vertical
     * * t - 0.5 * g * t²
     * 
     * We sample the trajectory and find where it's closest to (goal_x, goal_y, goal_z). The
     * "residual" is the signed vertical distance at the horizontal closest approach.
     */
    private static void computeClosestApproach(ShotState state, double thetaH, double goalX,
            double goalY) {

        double lambda = state.horizontalSpeed;
        double vVert = state.verticalSpeed;
        double dirX = state.launchDirectionX;
        double dirY = state.launchDirectionY;

        // Release position
        double x0 = state.releasePoseX;
        double y0 = state.releasePoseY;
        double z0 = LAUNCH_HEIGHT;

        // Goal position
        double gx = goalX;
        double gy = goalY;
        double gz = GOAL_HEIGHT;

        // ---------------------------------------------------------------------
        // Find time when ball is at horizontal distance closest to goal
        // ---------------------------------------------------------------------

        // Horizontal distance from release to goal
        double horizDist = state.distance;

        // Time to reach that horizontal distance (with drag)
        // x_horiz = (1/k) * ln(1 + k * λ * t)
        // horizDist = (1/k) * ln(1 + k * λ * t)
        // t = (exp(k * horizDist) - 1) / (k * λ)

        double E = Math.exp(k * horizDist) - 1;
        double tAtGoal = E / (k * lambda);

        // Clamp to reasonable range
        tAtGoal = clamp(tAtGoal, 0, MAX_TRAJECTORY_TIME);

        // ---------------------------------------------------------------------
        // Compute position at this time
        // ---------------------------------------------------------------------

        // Horizontal position
        double horizTravel = (1.0 / k) * Math.log(1 + k * lambda * tAtGoal);
        double xAtGoal = x0 + horizTravel * dirX;
        double yAtGoal = y0 + horizTravel * dirY;

        // Vertical position
        double zAtGoal = z0 + vVert * tAtGoal - 0.5 * g * tAtGoal * tAtGoal;

        // ---------------------------------------------------------------------
        // Store results
        // ---------------------------------------------------------------------

        state.closestT = tAtGoal;
        state.closestX = xAtGoal;
        state.closestY = yAtGoal;
        state.closestZ = zAtGoal;

        // Residual: signed vertical distance
        // Positive = ball is above goal (need to increase hood angle to lower trajectory)
        // Negative = ball is below goal (need to decrease hood angle to raise trajectory)
        // Note: Higher hood angle → lower launch angle → lower trajectory
        state.residual = zAtGoal - gz;
    }

    // =========================================================================
    // PHYSICS HELPER METHODS
    // =========================================================================

    /**
     * Converts hood angle (mechanical) to launch angle (trajectory).
     * 
     * Hood angle: servo/mechanical position (40-80°) Launch angle: actual angle above horizontal
     * (10-50°)
     * 
     * Relationship: launch = 90° - hood
     */
    private static double hoodAngleToLaunchAngle(double hoodAngle) {
        return LAUNCH_ANGLE_OFFSET - hoodAngle;
    }

    /**
     * Computes velocity ratio A(θ) as function of hood angle.
     */
    private static double computeVelocityRatio(double thetaH) {
        double thetaArc = thetaH - THETA_FIRST_CONTACT;

        if (thetaArc <= 0)
            return 0;
        if (thetaArc >= THETA_SLIP)
            return A_MAX;

        return A_MAX * Math.sqrt(thetaArc / THETA_SLIP);
    }

    /**
     * Computes ball exit velocity.
     */
    private static double computeExitVelocity(double thetaH, double omegaFlywheel) {
        return computeVelocityRatio(thetaH) * omegaFlywheel * R_FLYWHEEL;
    }

    /**
     * Computes contact time as function of hood angle and flywheel speed.
     */
    private static double computeContactTime(double thetaH, double omegaFlywheel) {
        double thetaArc = thetaH - THETA_FIRST_CONTACT;
        if (thetaArc <= 0)
            return 0;

        double vMax = A_MAX * omegaFlywheel * R_FLYWHEEL;
        if (vMax <= 0)
            return Double.MAX_VALUE;

        if (thetaArc < THETA_SLIP) {
            return 2 * R_COMBINED * Math.sqrt(thetaArc * THETA_SLIP) / vMax;
        } else {
            return R_COMBINED * (thetaArc + THETA_SLIP) / vMax;
        }
    }

    /**
     * Flywheel speed from distance (linear regression).
     */
    private static double flywheelSpeedFromDistance(double distance) {
        return REGRESSION_SLOPE * distance + REGRESSION_INTERCEPT;
    }

    // =========================================================================
    // TWIST INTEGRATION
    // =========================================================================

    /**
     * SE(2) exponential map for robot motion prediction.
     */
    private static double[] twistIntegrate(double x0, double y0, double theta0, double vxField,
            double vyField, double omega, double dt) {

        double dtheta = omega * dt;
        double newHeading = wrapAngle(theta0 + dtheta);

        // Convert to body frame
        double cosInit = Math.cos(theta0);
        double sinInit = Math.sin(theta0);
        double vxBody = vxField * cosInit + vyField * sinInit;
        double vyBody = -vxField * sinInit + vyField * cosInit;

        double dxBody, dyBody;

        if (Math.abs(omega) < 1e-9) {
            dxBody = vxBody * dt;
            dyBody = vyBody * dt;
        } else {
            double sinDtheta = Math.sin(dtheta);
            double cosDtheta = Math.cos(dtheta);
            dxBody = (vxBody * sinDtheta - vyBody * (1 - cosDtheta)) / omega;
            dyBody = (vxBody * (1 - cosDtheta) + vyBody * sinDtheta) / omega;
        }

        // Back to field frame
        double dxField = dxBody * cosInit - dyBody * sinInit;
        double dyField = dxBody * sinInit + dyBody * cosInit;

        return new double[] {x0 + dxField, y0 + dyField, newHeading};
    }

    // =========================================================================
    // UTILITY METHODS
    // =========================================================================

    private static double wrapAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
