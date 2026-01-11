package org.firstinspires.ftc.teamcode.shooter.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.Vector2D;

/**
 * ShooterSolver: Computes optimal hood angle, turret angle, and flywheel speed for shooting while
 * the robot is moving.
 * 
 * SOLVER APPROACH: ================ Rather than binary search (which assumes monotonicity), we use
 * Newton-Raphson on the "closest approach" residual:
 * 
 * 1. For a given θ_h, compute the full ball trajectory
 * 2. Find the point on the trajectory closest
 * to the goal
 * 3. r(θ_h) = signed distance from trajectory to goal at closest approach
 * 4. Compute
 * r'(θ_h) numerically
 * 5. Newton update: θ_h ← θ_h - r / r'
 * 6. Converged when |r| < tolerance
 * 
 * REAL-TIME UPDATE: ================= After solve() is called, updateTurretEstimate() can be called
 * continuously to refine the turret angle as the shot progresses. This compensates for prediction
 * errors by using the actual robot state and remaining time.
 */
public class ShooterSolver {

    // =========================================================================
    // TIMER AND STATE TRACKING
    // =========================================================================

    private static Timer timer = new Timer();

    /** Stored total time estimate from last solve() call (in seconds) */
    private static double storedTotalTime = 0;

    /** Stored hood angle from last solve() (needed for turret updates) */
    private static double storedHoodAngle = 0;

    /** Stored flywheel speed from last solve() (needed for turret updates) */
    private static double storedFlywheelSpeed = 0;

    /** Stored goal position from last solve() */
    private static double storedGoalX = 0;
    private static double storedGoalY = 0;

    /** Flag indicating if we have a valid solution to update */
    private static boolean hasSolution = false;

    // =========================================================================
    // PHYSICAL CONSTANTS
    // =========================================================================

    /** Gravity in inches/s² */
    private static final double g = 386.0885;

    /** Air density in slugs/in³ */
    private static final double rho = 1.376e-6;

    /** Drag coefficient (measured) */
    private static final double CD = 0.534;

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

    /**
     * Offset of turret pivot from robot center, in ROBOT FRAME (inches). Robot frame: +x = forward,
     * +y = left TODO: Measure actual values
     */
    private static final Vector2D SHOOTER_OFFSET = new Vector2D(0.0, 0.0);

    /**
     * Height of "shooter center" above ground (inches). "Shooter center" is defined as ball center
     * position when θ_hood = 0. TODO: Measure actual value
     */
    private static final double FLYWHEEL_HEIGHT = 12.0;

    /** Angle at which ball first contacts flywheel (radians) - negative means below horizontal */
    private static final double THETA_FIRST_CONTACT = Math.toRadians(-18.241);

    /** Minimum hood angle (radians) */
    private static final double MIN_HOOD_ANGLE = Math.toRadians(40);

    /** Maximum hood angle (radians) */
    private static final double MAX_HOOD_ANGLE = Math.toRadians(80);

    /**
     * Launch angle offset: launch_angle = LAUNCH_ANGLE_OFFSET - hood_angle
     */
    private static final double LAUNCH_ANGLE_OFFSET = Math.toRadians(90);

    /** Arc angle at which slip ends (radians) - CALIBRATE THIS */
    private static final double THETA_SLIP = Math.toRadians(60); // TODO: Calibrate

    /** Fixed delay before ball contacts flywheel (seconds) */
    private static final double T_INIT = 0.05; // TODO: Measure actual value

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
     * Container for turret update results (lightweight, for real-time updates).
     */
    public static class TurretUpdate {
        public final double turretAngle;
        public final double remainingTime;
        public final boolean isValid;

        public TurretUpdate(double turretAngle, double remainingTime) {
            this.turretAngle = turretAngle;
            this.remainingTime = remainingTime;
            this.isValid = true;
        }

        public TurretUpdate() {
            this.turretAngle = 0;
            this.remainingTime = 0;
            this.isValid = false;
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

        // Poses using PedroPathing Pose class
        Pose releasePose; // Robot x, y, heading at release
        Pose releaseVel; // Robot vx, vy, omega at release

        // Ball exit position (field frame, accounts for all offsets)
        double ballExitX; // Ball X position in field frame at exit
        double ballExitY; // Ball Y position in field frame at exit
        double ballExitZ; // Ball height (Z) at exit

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
     * 
     * This method resets the internal timer and stores the solution for use with
     * updateTurretEstimate().
     * 
     * @param robotPose Current Robot Pose (x, y, heading)
     * @param robotVel Current Robot Velocity (x=vx, y=vy, heading=omega)
     * @param goalX Target X coordinate
     * @param goalY Target Y coordinate
     * @return ShotSolution containing hood angle, turret angle, etc.
     */
    public static ShotSolution solve(Pose robotPose, Pose robotVel, double goalX, double goalY) {

        // Reset timer at the start of a new solution
        timer.resetTimer();

        // Store goal position for updates
        storedGoalX = goalX;
        storedGoalY = goalY;

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
            ShotState state = computeFullState(thetaH, robotPose, robotVel, goalX, goalY);

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
                // Store solution for updates
                storedTotalTime = state.tTotal;
                storedHoodAngle = thetaH;
                storedFlywheelSpeed = state.omegaFlywheel;
                hasSolution = true;

                return new ShotSolution(thetaH, state.turretAngle, state.omegaFlywheel,
                        state.tTotal, state.distance, r);
            }

            // -----------------------------------------------------------------
            // COMPUTE NUMERICAL DERIVATIVE dr/dθ_h
            // -----------------------------------------------------------------

            double thetaPlus = Math.min(thetaH + DERIVATIVE_DELTA, MAX_HOOD_ANGLE);
            double thetaMinus = Math.max(thetaH - DERIVATIVE_DELTA, MIN_HOOD_ANGLE);

            ShotState statePlus = computeFullState(thetaPlus, robotPose, robotVel, goalX, goalY);
            ShotState stateMinus = computeFullState(thetaMinus, robotPose, robotVel, goalX, goalY);

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
            // Store solution for updates
            storedTotalTime = bestState.tTotal;
            storedHoodAngle = bestThetaH;
            storedFlywheelSpeed = bestState.omegaFlywheel;
            hasSolution = true;

            return new ShotSolution(bestThetaH, bestState.turretAngle, bestState.omegaFlywheel,
                    bestState.tTotal, bestState.distance, bestResidual);
        }

        hasSolution = false;
        return new ShotSolution("Failed to converge. Best residual: " + bestResidual);
    }

    // =========================================================================
    // REAL-TIME TURRET UPDATE
    // =========================================================================

    /**
     * Updates the turret angle estimate during an active shot.
     * 
     * This method should be called continuously while the shooter is actively shooting (after
     * solve() has been called). It uses the current robot state and remaining time to refine the
     * turret angle, compensating for prediction errors.
     * 
     * If the estimated time has elapsed (remaining time <= 0), this method will call solve() to
     * compute a completely new solution.
     * 
     * @param currentPose Current robot pose (x, y, heading)
     * @param currentVel Current robot velocity (vx, vy, omega)
     * @return TurretUpdate containing the updated turret angle and remaining time
     */
    public static TurretUpdate updateTurretEstimate(Pose currentPose, Pose currentVel) {
        // Check if we have a valid solution to update
        if (!hasSolution) {
            return new TurretUpdate();
        }

        // Calculate elapsed time
        // NOTE: Assumes timer.getElapsedTime() returns MILLISECONDS
        // If PedroPathing Timer returns seconds, remove the / 1000.0
        double elapsedTime = timer.getElapsedTime() / 1000.0;
        double remainingTime = storedTotalTime - elapsedTime;

        // If time has elapsed, the ball should have been released
        // Return invalid - caller should call solve() explicitly for a new shot
        if (remainingTime <= 0) {
            // Clear the solution since the shot window has passed
            hasSolution = false;
            return new TurretUpdate();
        }

        // ---------------------------------------------------------------------
        // PREDICT RELEASE POSE WITH REMAINING TIME
        // ---------------------------------------------------------------------

        // Predict where the robot will be at release
        Pose predictedReleasePose = twistIntegrate(currentPose, currentVel, remainingTime);

        // Compute predicted velocity at release (rotate by heading change)
        double dtheta = currentVel.getHeading() * remainingTime;
        double cos = Math.cos(dtheta);
        double sin = Math.sin(dtheta);
        double vx = currentVel.getX();
        double vy = currentVel.getY();
        double newVx = vx * cos - vy * sin;
        double newVy = vx * sin + vy * cos;
        Pose predictedReleaseVel = new Pose(newVx, newVy, currentVel.getHeading());

        // ---------------------------------------------------------------------
        // SOLVE TURRET ANGLE WITH STORED HOOD/FLYWHEEL VALUES
        // ---------------------------------------------------------------------

        double turretAngle = computeTurretAngleOnly(storedHoodAngle, storedFlywheelSpeed,
                predictedReleasePose, predictedReleaseVel, storedGoalX, storedGoalY);

        if (Double.isNaN(turretAngle)) {
            // Turret solve failed - return invalid
            return new TurretUpdate();
        }

        return new TurretUpdate(turretAngle, remainingTime);
    }

    /**
     * Computes only the turret angle without full state computation.
     * 
     * This is a lightweight version of solveTurretAngle for real-time updates. Uses stored hood
     * angle and flywheel speed.
     * 
     * @return Turret angle in radians, or NaN if no valid solution
     */
    private static double computeTurretAngleOnly(double thetaH, double omegaFlywheel,
            Pose releasePose, Pose releaseVel, double goalX, double goalY) {

        // 1. Calculate Shooter Physics
        double launchAngle = hoodAngleToLaunchAngle(thetaH);
        double vShooter = computeExitVelocity(thetaH, omegaFlywheel);

        // Horizontal component of shot speed
        double vHorizontal = vShooter * Math.cos(launchAngle);

        // Validity check: horizontal speed must be positive
        if (vHorizontal <= 1e-6) {
            return Double.NaN;
        }

        // 2. Compute shooter center position (with SHOOTER_OFFSET)
        double cosRel = Math.cos(releasePose.getHeading());
        double sinRel = Math.sin(releasePose.getHeading());
        double shooterOffsetRelX = SHOOTER_OFFSET.getX() * cosRel - SHOOTER_OFFSET.getY() * sinRel;
        double shooterOffsetRelY = SHOOTER_OFFSET.getX() * sinRel + SHOOTER_OFFSET.getY() * cosRel;

        double shooterCenterX = releasePose.getX() + shooterOffsetRelX;
        double shooterCenterY = releasePose.getY() + shooterOffsetRelY;

        // 3. Goal Geometry (from shooter center)
        double dx = goalX - shooterCenterX;
        double dy = goalY - shooterCenterY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        if (distToGoal < 1e-6) {
            return Double.NaN;
        }

        // 4. Analytic Solution
        double rVelX = releaseVel.getX();
        double rVelY = releaseVel.getY();

        // Robot's tangential velocity relative to goal
        double crossProduct = rVelX * dy - rVelY * dx;
        double vRobotTangential = crossProduct / distToGoal;

        // Validity Check
        if (Math.abs(vRobotTangential) > vHorizontal) {
            return Double.NaN;
        }

        // Calculate angle offset (beta)
        double sinBeta = -vRobotTangential / vHorizontal;
        double beta = Math.asin(sinBeta);

        // 5. Compute Turret Angle
        double angleToGoal = Math.atan2(dy, dx);
        double turretFieldHeading = angleToGoal + beta;

        // Convert to Robot Frame
        return wrapAngle(turretFieldHeading - releasePose.getHeading());
    }

    /**
     * Returns the remaining time until expected ball release.
     * 
     * @return Remaining time in seconds, or 0 if no active solution
     */
    public static double getRemainingTime() {
        if (!hasSolution) {
            return 0;
        }
        double elapsedTime = timer.getElapsedTime() / 1000.0;
        return Math.max(0, storedTotalTime - elapsedTime);
    }

    /**
     * Returns whether there is an active solution being tracked.
     */
    public static boolean hasActiveSolution() {
        return hasSolution;
    }

    /**
     * Clears the current solution. Call this when aborting a shot.
     */
    public static void clearSolution() {
        hasSolution = false;
        storedTotalTime = 0;
    }

    // =========================================================================
    // COMPUTE FULL STATE FOR A GIVEN HOOD ANGLE
    // =========================================================================

    private static ShotState computeFullState(double thetaH, Pose robotPose, Pose robotVel,
            double goalX, double goalY) {

        ShotState state = new ShotState();

        // ---------------------------------------------------------------------
        // INNER LOOP: Solve for (ω_f, t_contact, d)
        // ---------------------------------------------------------------------

        if (!solveInnerLoop(state, thetaH, robotPose, robotVel, goalX, goalY)) {
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
     * Solves for flywheel speed, contact time, and distance via fixed-point iteration. Distance is
     * computed from shooter center (robot + SHOOTER_OFFSET) to goal.
     */
    private static boolean solveInnerLoop(ShotState state, double thetaH, Pose robotPose,
            Pose robotVel, double goalX, double goalY) {

        // Compute initial shooter center position (robot + SHOOTER_OFFSET rotated to field frame)
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());
        double shooterOffsetFieldX = SHOOTER_OFFSET.getX() * cosH - SHOOTER_OFFSET.getY() * sinH;
        double shooterOffsetFieldY = SHOOTER_OFFSET.getX() * sinH + SHOOTER_OFFSET.getY() * cosH;

        double shooterX = robotPose.getX() + shooterOffsetFieldX;
        double shooterY = robotPose.getY() + shooterOffsetFieldY;

        // Initial guess based on current distance from shooter center
        double dxInitial = goalX - shooterX;
        double dyInitial = goalY - shooterY;
        double distInitial = Math.sqrt(dxInitial * dxInitial + dyInitial * dyInitial);
        double omegaF = flywheelSpeedFromDistance(distInitial);

        for (int i = 0; i < MAX_INNER_ITERATIONS; i++) {

            // Compute contact time
            double tContact = computeContactTime(thetaH, omegaF);
            double tTotal = T_INIT + tContact;

            // Compute release pose (Predict robot motion during shot windup/transit)
            Pose releasePose = twistIntegrate(robotPose, robotVel, tTotal);

            // Compute shooter center at release (robot + SHOOTER_OFFSET rotated)
            double cosRel = Math.cos(releasePose.getHeading());
            double sinRel = Math.sin(releasePose.getHeading());
            double shooterOffsetRelX =
                    SHOOTER_OFFSET.getX() * cosRel - SHOOTER_OFFSET.getY() * sinRel;
            double shooterOffsetRelY =
                    SHOOTER_OFFSET.getX() * sinRel + SHOOTER_OFFSET.getY() * cosRel;

            double shooterRelX = releasePose.getX() + shooterOffsetRelX;
            double shooterRelY = releasePose.getY() + shooterOffsetRelY;

            // Compute distance to goal from shooter center at release
            double dx = goalX - shooterRelX;
            double dy = goalY - shooterRelY;
            double dist = Math.sqrt(dx * dx + dy * dy);

            // New flywheel speed required for this distance
            double omegaFNew = flywheelSpeedFromDistance(dist);

            // Check convergence
            if (Math.abs(omegaFNew - omegaF) < OMEGA_TOLERANCE) {
                // Store results
                state.omegaFlywheel = omegaFNew;
                state.tContact = tContact;
                state.tTotal = tTotal;
                state.distance = dist;
                state.releasePose = releasePose;

                // Compute Robot velocity at release (rotate velocity vector by change in heading)
                double dtheta = robotVel.getHeading() * tTotal; // Omega * t
                double cos = Math.cos(dtheta);
                double sin = Math.sin(dtheta);

                double vx = robotVel.getX();
                double vy = robotVel.getY();

                // Rotate velocity vector
                double newVx = vx * cos - vy * sin;
                double newVy = vx * sin + vy * cos;

                // Store in Pose (x=vx, y=vy, heading=omega)
                state.releaseVel = new Pose(newVx, newVy, robotVel.getHeading());

                return true;
            }

            omegaF = omegaFNew;
        }

        return false; // Failed to converge
    }

    // =========================================================================
    // TURRET COMPENSATION (Analytic Sine Rule)
    // =========================================================================

    private static boolean solveTurretAngle(ShotState state, double thetaH, double goalX,
            double goalY) {

        // 1. Calculate Shooter Physics
        double launchAngle = hoodAngleToLaunchAngle(thetaH);
        double vShooter = computeExitVelocity(thetaH, state.omegaFlywheel);

        // Horizontal component of shot speed (relative to robot)
        double vHorizontal = vShooter * Math.cos(launchAngle);
        double vVertical = vShooter * Math.sin(launchAngle);

        state.verticalSpeed = vVertical;

        // Validity check: horizontal speed must be positive
        if (vHorizontal <= 1e-6) {
            state.turretValid = false;
            return false;
        }

        // 2. Compute shooter center position at release
        double cosRel = Math.cos(state.releasePose.getHeading());
        double sinRel = Math.sin(state.releasePose.getHeading());
        double shooterOffsetRelX = SHOOTER_OFFSET.getX() * cosRel - SHOOTER_OFFSET.getY() * sinRel;
        double shooterOffsetRelY = SHOOTER_OFFSET.getX() * sinRel + SHOOTER_OFFSET.getY() * cosRel;

        double shooterCenterX = state.releasePose.getX() + shooterOffsetRelX;
        double shooterCenterY = state.releasePose.getY() + shooterOffsetRelY;

        // 3. Goal Geometry (from shooter center)
        double dx = goalX - shooterCenterX;
        double dy = goalY - shooterCenterY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        if (distToGoal < 1e-6) {
            state.turretValid = false;
            return false;
        }

        // 4. Analytic Solution
        // We need the shooter's tangential component to cancel the robot's tangential component.

        // Robot Velocity at release
        double rVelX = state.releaseVel.getX();
        double rVelY = state.releaseVel.getY();

        // Calculate Robot's tangential velocity relative to the goal line using 2D cross product.
        // v_tan = (v_robot x d_goal) / |d_goal| = (vx*dy - vy*dx) / dist
        double crossProduct = rVelX * dy - rVelY * dx;
        double vRobotTangential = crossProduct / distToGoal;

        // Validity Check: Can the shooter physically cancel this sideways motion?
        if (Math.abs(vRobotTangential) > vHorizontal) {
            state.turretValid = false;
            return false;
        }

        // Calculate the angle offset (beta) required.
        // v_shot * sin(beta) + v_robot_tan = 0 -> sin(beta) = -v_robot_tan / v_shot
        double sinBeta = -vRobotTangential / vHorizontal;
        double beta = Math.asin(sinBeta);

        // 5. Compute Turret Angle
        // Turret Angle (Field Frame) = AngleToGoal + Offset
        double angleToGoal = Math.atan2(dy, dx);
        double turretFieldHeading = angleToGoal + beta;

        // Convert to Robot Frame
        state.turretAngle = wrapAngle(turretFieldHeading - state.releasePose.getHeading());

        // 6. Compute ball exit position (with horizontal offset in turret direction)
        double horizOffset = computeBallExitHorizontalOffset(thetaH);
        double turretDirX = Math.cos(turretFieldHeading);
        double turretDirY = Math.sin(turretFieldHeading);

        state.ballExitX = shooterCenterX + horizOffset * turretDirX;
        state.ballExitY = shooterCenterY + horizOffset * turretDirY;
        state.ballExitZ = computeBallExitHeight(thetaH);

        // 7. Recompute distance from ball exit to goal
        double dxBall = goalX - state.ballExitX;
        double dyBall = goalY - state.ballExitY;
        double distBallToGoal = Math.sqrt(dxBall * dxBall + dyBall * dyBall);

        // Update distance in state (for trajectory calculation)
        state.distance = distBallToGoal;

        // Calculate Resultant Speed (Lambda) toward goal for trajectory calculation.
        // v_radial_total = v_robot_radial + v_shot_radial
        double dotProduct = rVelX * dx + rVelY * dy;
        double vRobotRadial = dotProduct / distToGoal;

        // v_shot_radial = v_shot * cos(beta)
        double vShooterRadial = vHorizontal * Math.cos(beta);

        state.horizontalSpeed = vRobotRadial + vShooterRadial;

        // Validity check: resultant speed toward goal must be positive
        // (ball must actually be moving toward goal, not away)
        if (state.horizontalSpeed <= 0) {
            state.turretValid = false;
            return false;
        }

        // The ball travels directly at the goal (use direction from ball exit, not shooter center)
        state.launchDirectionX = dxBall / distBallToGoal;
        state.launchDirectionY = dyBall / distBallToGoal;
        state.turretValid = true;

        return true;
    }

    // =========================================================================
    // TRAJECTORY ANALYSIS
    // =========================================================================

    private static void computeClosestApproach(ShotState state, double thetaH, double goalX,
            double goalY) {

        double lambda = state.horizontalSpeed;
        double vVert = state.verticalSpeed;
        double dirX = state.launchDirectionX;
        double dirY = state.launchDirectionY;

        // Safety check: lambda should be positive (checked in solveTurretAngle)
        // but add a fallback to prevent division issues
        if (lambda <= 1e-6) {
            state.residual = Double.MAX_VALUE;
            return;
        }

        // Ball exit position (computed in solveTurretAngle with all offsets)
        double x0 = state.ballExitX;
        double y0 = state.ballExitY;
        double z0 = state.ballExitZ;

        // Goal position
        double gz = GOAL_HEIGHT;

        // Time to reach goal horizontal distance (with drag)
        double horizDist = state.distance;
        double E = Math.exp(k * horizDist) - 1;
        double tAtGoal = E / (k * lambda);

        // Clamp to reasonable range
        tAtGoal = clamp(tAtGoal, 0, MAX_TRAJECTORY_TIME);

        // Compute position at this time
        double horizTravel = (1.0 / k) * Math.log(1 + k * lambda * tAtGoal);
        double xAtGoal = x0 + horizTravel * dirX;
        double yAtGoal = y0 + horizTravel * dirY;
        double zAtGoal = z0 + vVert * tAtGoal - 0.5 * g * tAtGoal * tAtGoal;

        state.closestT = tAtGoal;
        state.closestX = xAtGoal;
        state.closestY = yAtGoal;
        state.closestZ = zAtGoal;
        state.residual = zAtGoal - gz;
    }

    // =========================================================================
    // PHYSICS HELPER METHODS
    // =========================================================================

    private static double hoodAngleToLaunchAngle(double hoodAngle) {
        return LAUNCH_ANGLE_OFFSET - hoodAngle;
    }

    private static double computeVelocityRatio(double thetaH) {
        double thetaArc = thetaH - THETA_FIRST_CONTACT;
        if (thetaArc <= 0)
            return 0;
        if (thetaArc >= THETA_SLIP)
            return A_MAX;
        return A_MAX * Math.sqrt(thetaArc / THETA_SLIP);
    }

    private static double computeExitVelocity(double thetaH, double omegaFlywheel) {
        return computeVelocityRatio(thetaH) * omegaFlywheel * R_FLYWHEEL;
    }

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

    private static double flywheelSpeedFromDistance(double distance) {
        return REGRESSION_SLOPE * distance + REGRESSION_INTERCEPT;
    }

    /**
     * Computes the ball exit height based on hood angle. Height = FLYWHEEL_HEIGHT + R_COMBINED *
     * sin(θ_hood)
     */
    private static double computeBallExitHeight(double thetaH) {
        return FLYWHEEL_HEIGHT + R_COMBINED * Math.sin(thetaH);
    }

    /**
     * Computes the horizontal offset of ball exit from "center of shooter" in turret direction.
     * Offset = R_COMBINED * (1 - cos(θ_hood))
     */
    private static double computeBallExitHorizontalOffset(double thetaH) {
        return R_COMBINED * (1 - Math.cos(thetaH));
    }

    // =========================================================================
    // TWIST INTEGRATION (Updated for Pose)
    // =========================================================================

    /**
     * SE(2) exponential map for robot motion prediction.
     * 
     * @param startPose Current Pose
     * @param vel Current Velocity (x, y, heading=omega)
     * @param dt Time delta
     * @return Predicted Pose
     */
    private static Pose twistIntegrate(Pose startPose, Pose vel, double dt) {
        double x0 = startPose.getX();
        double y0 = startPose.getY();
        double theta0 = startPose.getHeading();

        double vxField = vel.getX();
        double vyField = vel.getY();
        double omega = vel.getHeading();

        double dtheta = omega * dt;
        double newHeading = wrapAngle(theta0 + dtheta);

        // Convert field velocity to body frame for twist calculation
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

        return new Pose(x0 + dxField, y0 + dyField, newHeading);
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
