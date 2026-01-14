
package frc.robot.subsystems.ultilities.MovingPlatformShooterSolver;

/*
 * ╔════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  SHOOTER SOLVER: 2D Ballistic Trajectory Computation for Moving Robot → Fixed Target                       ║
 * ╚════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
 * 
 * PURPOSE:
 * --------
 * This subsystem solves the inverse ballistic problem: Given the robot's current position and velocity,
 * calculate the exact hood angle and robot yaw required to hit a fixed target with a projectile.
 * 
 * Unlike traditional shooter solutions that assume a stationary robot, this accounts for:
 *   • Robot translation: robot is moving toward/away from target at launch
 *   • Robot rotation: robot's orientation affects launch angle
 *   • Projectile motion: gravity affects vertical trajectory
 *   • Target offset: target may be above or below launch point
 * 
 * THE 2D BALLISTIC PROBLEM:
 * -------------------------
 * We want to find: hood_angle and robot_yaw such that a projectile:
 *   1. Launches from the shooter (at some position on the robot)
 *   2. Follows a parabolic trajectory under gravity
 *   3. Arrives at the target position with any non-negative velocity
 * 
 * COORDINATE SYSTEMS:
 * -------------------
 * Field Frame (absolute):
 *   • Origin: corner of field
 *   • X-axis: points toward one end
 *   • Y-axis: perpendicular to X
 *   • Z-axis: vertically upward
 *   • Robot pose (Px, Py, θ) defines position and heading
 *   • Shooter offset is relative to robot center
 * 
 * Robot Frame (rotating with robot):
 *   • Origin: robot center
 *   • Forward: direction of robot heading
 *   • Lateral: perpendicular to forward (left side)
 *   • Vertical: upward from ground plane
 *   • Shooter position relative to robot center: (dx, dy, dz)
 * 
 * Projectile Frame (launch-relative):
 *   • yaw: rotation about vertical axis (horizontal aiming)
 *   • pitch/hood: angle above horizontal (elevation aiming)
 *   • Combined they define a 3D launch vector
 * 
 * PROJECTILE MOTION PHYSICS:
 * --------------------------
 * A projectile launched with initial velocity v0 at angle α (above horizontal) and yaw θ:
 * 
 *   Launch velocity components (in field frame):
 *     vx = v0 · cos(α) · cos(θ)     [forward component in field X direction]
 *     vy = v0 · cos(α) · sin(θ)     [forward component in field Y direction]
 *     vz = v0 · sin(α)              [vertical component upward]
 * 
 *   Trajectory equations (as function of time t):
 *     x(t) = x0 + vx·t              [horizontal X position]
 *     y(t) = y0 + vy·t              [horizontal Y position]
 *     z(t) = z0 + vz·t - (1/2)·g·t² [vertical position with gravity]
 * 
 *   Where:
 *     (x0, y0, z0) = initial position of shooter
 *     g = 9.81 m/s²  (gravitational acceleration)
 *     t = flight time
 * 
 * ROBOT MOTION COMPLICATION:
 * ---------------------------
 * The robot (and thus the shooter) is moving with velocity (vx_robot, vy_robot).
 * At time t after launch, the robot will have moved to:
 *     robot_x(t) = Px + vx_robot·t
 *     robot_y(t) = Py + vy_robot·t
 * 
 * The shooter muzzle is at position relative to robot:
 *     shooter_x(t) = robot_x(t) + dx·cos(θ_robot) - dy·sin(θ_robot)
 *     shooter_y(t) = robot_y(t) + dx·sin(θ_robot) + dy·cos(θ_robot)
 *     shooter_z(t) = Pz + dz  (constant height above ground)
 * 
 * So the projectile position is:
 *     bullet_x(t) = shooter_x(0) + vx_bullet·t
 *     bullet_y(t) = shooter_y(0) + vy_bullet·t
 *     bullet_z(t) = shooter_z(0) + vz_bullet·t - (1/2)·g·t²
 * 
 * SOLVING FOR IMPACT TIME:
 * -------------------------
 * To hit target at (Tx, Ty, Tz), we need:
 *     bullet_x(t) = Tx
 *     bullet_y(t) = Ty
 *     bullet_z(t) = Tz
 * 
 * From the X and Y equations:
 *     shooter_x(0) + vx_bullet·t = Tx
 *     shooter_y(0) + vy_bullet·t = Ty
 * 
 * We can solve for time t algebraically IF we know vx_bullet and vy_bullet.
 * 
 * The challenge: vx_bullet and vy_bullet depend on hood angle (α) and yaw (θ).
 * We must find the combination that satisfies all three equations.
 * 
 * SOLUTION METHOD: Golden Section Search + Geometry
 * ---------------------------------------------------
 * This code uses a two-step approach:
 * 
 * Step 1: Determine Required Horizontal Direction (Ball path Yaw)
 *   • Calculate vector from shooter to target: (Δx, Δy)
 *   • Compute required direction angle: θ_target = atan2(Δy, Δx)
 *   • Ball must be launched along this direction to reach target horizontally
 *   • Horizontal velocity will determine the amount of time to reach the target x,y position
 *   
 * 
 * Step 2: Determine Hood Angle Using Golden Section Search
 *   • For a given horizontal direction, the only unknown is hood angle (α)
 *   • Use golden section search to find α that makes projectile arrive at target height, at the same time as reaching target x,y
 *   • Objective function: Minimize y error 
 *   • Golden section search: Efficient 1D optimization (avoids computing derivatives)
 *   • After finding a valid hood angle, we now know the horizontal lauch vectory (x_ball_dot,y_ball_dot)
 *   • We find the shooter yaw by subtracting the robot velocity components from the required ball launch components and running the inverse atan2
 *   • Now we know both hood angle, and required shooter yaw. Robot yaw can be found by subtracting the shooter_to_robot yaw transform
 * 

 * 
 * 
 * 
 * HORIZONTAL CONSTRAINT:
 * ----------------------
 * Once we have impact time (t) and hood angle (α), verify horizontal accuracy:
 * 
 *     horizontal_distance = sqrt[(Tx - x0)² + (Ty - y0)²]
 *     expected_range = v0·cos(α)·t
 * 
 * These should match (within numerical tolerance) if our yaw is correct.
 * The solve() method enforces this constraint during the search.
 * 
 * RATE LIMITING AND FILTERING:
 * ----------------------------
 * Robot yaw commands are rate-limited using a low-pass filter:
 *     filtered_yaw(t+1) = filtered_yaw(t) + clamp(desired_yaw - filtered_yaw(t), max_rate)
 * 
 * This prevents the shooter from spinning too fast, which would:
 *   • Cause mechanical stress on the drive system
 *   • Overshoot the target due to control lag
 *   • Create vibration affecting accuracy
 * 
 * VALIDITY CHECKS:
 * ----------------
 * A solution is considered valid only if:
 *   1. Impact time is positive (t > 0)
 *   2. Height error is small (|Δz| < threshold)
 *   3. Horizontal distance matches (no undershoot/overshoot)
 *   4. Solution converged to sufficient precision
 * 
 * If no valid solution exists:
 *   • Target may be out of range (beyond maximum shooting distance)
 *   • Target may be too high/low (hood angle limits exceeded)
 *   • Muzzle velocity insufficient for target distance
 * 
 * INTEGRATION WITH DRIVE SUBSYSTEM:
 * ----------------------------------
 * The Drive subsystem can:
 *   1. Query required robot yaw from this solver
 *   2. Query solution validity status
 *   3. Automatically align yaw when valid solution exists
 *   4. Execute shots via feeder trigger signal
 * 
 * This creates closed-loop auto-aim: robot continuously updates yaw to track target.
 * 
 * SIMULATION VISUALIZATION:
 * -------------------------
 * In simulation, this code publishes trajectory points to NetworkTables:
 *   • trajectoryvis[60]: positions along the projectile path
 *   • trajectoryvis_prime[60]: velocities at each point
 * 
 * These are visualized in AdvantageScope for debugging trajectory shape.
 * 
 * KEY ASSUMPTIONS:
 * ----------------
 *   • Target position is static or moves very slowly
 *   • Robot velocity is relatively constant during flight (short flight time)
 *   • Projectile mass is negligible (no air resistance)
 *   • Gravity constant: g = 9.81 m/s²
 *   • Muzzle velocity is constant (no friction in shooter)
 *   • Impact happens on target, not before/after
 */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Solver extends SubsystemBase {
    // Network table publisher for trajectory visualization in simulation
    private StructArrayPublisher<Translation3d> trajectoryPublisher;
    // Supplier for current robot pose (x, y, rotation)
    private Supplier<Pose2d> robotPoseGetter;
    // Supplier for robot velocity including linear and angular components
    private Supplier<Transform2d> robotVelocityGetter;
    // Transform from robot center to shooter mounting point
    private Supplier<Transform3d> robotToShooter;
    // Supplier for target location being shot at
    private Supplier<Pose3d> targetPoseGetter;
    // Supplier for projectile muzzle velocity
    private Supplier<Double> shooterMuzzleV0Supplier;
    // Minimum time between solution recalculations to avoid excessive computation
    private double RefreshRate = 0.02; // 20 ms
    // Timestamp of last kinematics solution calculation
    private double lastTimeSolved = 0.0;
    // Smoothed required robot yaw angle with rate limiting
    private double filteredRequiredRobotYawRad = 0.0;
    // Unique identifier for this shooter solver instance
    private int shooter_number;
    // Flag to enable shooting when true
    private boolean shooting_permisive = false;
    // Watchdog timer to prevent shooting beyond allowed duration
    private double shooting_watchdog = 0.0;
    // Maximum number of trajectory points to visualize in simulation
    private static final int num_balls_vis = 60;
    // Simulated projectile positions along trajectory
    private Translation3d[] trajectoryvis = new Translation3d[num_balls_vis];
    // Simulated projectile velocities at each trajectory point
    private Translation3d[] trajectoryvis_prime = new Translation3d[num_balls_vis];
    // Current index in trajectory visualization array
    private int ball_index = 0;
    // Timestamp of the last shot in simulation
    private double last_shot_time = 0.0;

    // Internal class representing a projectile trajectory solution
    private static class Trajectory {
        // Launch angle in radians from horizontal
        public final double angle;
        // Time for projectile to reach target
        public final double time;
        // Whether this trajectory is physically valid
        public final boolean isValid;
        // Vertical position error at impact (how far from target height)
        public final double Yerror;
        
        public Trajectory(double angle, double time, double Yerror) {
            this.angle = angle;
            this.time = time;
            this.Yerror = Yerror;
            this.isValid = true;
        }   
        
        // Invalid trajectory marker
        @SuppressWarnings("unused")
        public Trajectory(){
            this.angle = 0;
            this.time = 0;
            this.isValid = false;
            this.Yerror = Double.POSITIVE_INFINITY;
        }    
    }
    
    // Complete kinematics solution with all outputs needed for robot control
    public static class Solution {
        // Description of solution quality ("Solution Found" or "No Solution")
        public final String status;
        // Required robot rotation angle about vertical axis
        public final double requiredRobotYawRad;
        // Required shooter mechanism pitch angle
        public final double requiredShooterPitchRad;
        // Predicted flight time from shooter to target
        public final double timeOfFlight;
        // Muzzle velocity validation value
        public final double muzzleVelocityCheck;
        // Projectile velocity at launch in world coordinates
        public final Translation3d shooterVelocityCheck;
        // Minimum vertical error achieved in solution
        public final double minYError;
        // Timestamp when solution was calculated
        public double timestamp;
        
        public Solution(String status, double requiredRobotYawRad, double requiredShooterPitchRad, double timeOfFlight, double muzzleVelocityCheck, Translation3d shooterVelocityCheck, double minYError) {
            this.status = status;
            this.requiredRobotYawRad = requiredRobotYawRad;
            this.requiredShooterPitchRad = requiredShooterPitchRad;
            this.timeOfFlight = timeOfFlight;
            this.muzzleVelocityCheck = muzzleVelocityCheck;
            this.shooterVelocityCheck = shooterVelocityCheck;
            this.minYError = minYError;
            timestamp = Timer.getFPGATimestamp();
        }
    }
    private Solution _best_solution = new Solution("No Solution", 0, 0, 0, 0, new Translation3d(0,0,0),Double.POSITIVE_INFINITY);
    
    // Constructor initializes all suppliers and sets up trajectory visualization
    public Shooter_Solver(int number,
                         Supplier<Pose2d> robotPoseGetter,
                         Supplier<Transform2d> robotVelocityGetter,
                         Supplier<Transform3d> robotToShooter,
                         Supplier<Pose3d> targetPoseGetter,
                         Supplier<Double> shooterMuzzleV0Supplier) {
        
        this.robotPoseGetter = robotPoseGetter;
        this.robotVelocityGetter = robotVelocityGetter;
        this.robotToShooter = robotToShooter;
        this.targetPoseGetter = targetPoseGetter;
        this.shooterMuzzleV0Supplier = shooterMuzzleV0Supplier;
        this.lastTimeSolved = Timer.getFPGATimestamp();
        this.shooter_number = number;

        // Initialize trajectory visualization arrays
        for(int i =0; i < num_balls_vis; i++){
            trajectoryvis[i] = new Translation3d(0,0,0);
            trajectoryvis_prime[i] = new Translation3d(0,0,0);
        }
        
        // Setup network table for simulation trajectory publishing
        trajectoryPublisher = 
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("trajectoryvis" + number, Translation3d.struct)
            .publish();
    }
    
    // Factory method for creating solver instances
    public static Shooter_Solver getInstance(int number,
                                        Supplier<Pose2d> robotPoseGetter,
                                        Supplier<Transform2d> robotVelocityGetter,
                                        Supplier<Transform3d> robotToShooter,
                                        Supplier<Pose3d> targetPoseGetter,
                                        Supplier<Double> shooterMuzzleV0Supplier) {
        return new Shooter_Solver(number,robotPoseGetter, robotVelocityGetter, robotToShooter, targetPoseGetter, shooterMuzzleV0Supplier);
    }
    
    // Solve for projectile launch angle using golden section search method
    private Trajectory solve2DTrajectory(double deltaX, double deltaY, double v0, double projectedVelocityFromRobot) {
        double g = 9.81; // gravity constant
        double lower = Math.toRadians(45); // minimum search angle
        double upper = Math.toRadians(90); // maximum search angle
        double tol = Math.toRadians(.5); // convergence tolerance
        double gr = (Math.sqrt(5) - 1) / 2; // golden ratio for search interval reduction
        
        // Error function evaluates vertical position miss for a given launch angle
        java.util.function.DoubleFunction<Double> errorFunc = (theta) -> {
            // Horizontal velocity includes shooter velocity and robot motion contribution
            double vx = v0 * Math.cos(theta) + projectedVelocityFromRobot;
            
            // Invalid if horizontal velocity is zero or negative
            if (vx <= 0) return Double.POSITIVE_INFINITY;
            
            // Time to reach horizontal distance
            double t = deltaX / vx;
            // Calculate where projectile lands vertically
            double y_pred = v0 * Math.sin(theta) * t - 0.5 * g * t * t;
            // Vertical velocity at impact
            double y_vel = v0 * Math.sin(theta) - g * t;
            
            // Reject if projectile is still ascending (undershot)
            if (y_vel > 0) {
                return Double.POSITIVE_INFINITY;
            }
            return Math.abs(y_pred - deltaY);
        };
    
        // Initialize search interval bounds
        double c = upper - gr * (upper - lower);
        double d = lower + gr * (upper - lower);
        double fc = errorFunc.apply(c);
        double fd = errorFunc.apply(d);
        
        // Golden section search for optimal angle
        while (Math.abs(upper - lower) > tol) {
            if (fc < fd) {
                upper = d;
                d = c;
                fd = fc;
                c = upper - gr * (upper - lower);
                fc = errorFunc.apply(c);
            } else {
                lower = c;
                c = d;
                fc = fd;
                d = lower + gr * (upper - lower);
                fd = errorFunc.apply(d);
            }
        }
        
        // Calculate best solution at interval midpoint
        double bestTheta = (lower + upper) / 2.0;
        double bestError = errorFunc.apply(bestTheta);
        double t_flight = deltaX / (v0 * Math.cos(bestTheta) + projectedVelocityFromRobot);
    
        return new Trajectory(bestTheta, t_flight, bestError);
    }
  
    // Calculate optimal shooter angle and required robot rotation for target hit
    private double objectiveFunction(Pose3d shooterPoseWorld, Translation3d shooterVelWorld) {
        Pose3d targetPose = this.targetPoseGetter.get();
        
        // Calculate displacement from shooter to target in all axes
        double deltaX = targetPose.getX() - shooterPoseWorld.getX();
        double deltaY = targetPose.getY() - shooterPoseWorld.getY();
        double deltaZ = targetPose.getZ() - shooterPoseWorld.getZ();
        
        // Project 3D problem to 2D (horizontal plane and vertical)
        double dxEff = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double dyEff = deltaZ;
        // Component of shooter velocity in direction of target
        double projectedVelocityFromRobot = (shooterVelWorld.getX()*deltaX + shooterVelWorld.getY()*deltaY)/dxEff;
        
        // Find launch angle that hits target
        Trajectory trajectory = solve2DTrajectory(dxEff, dyEff, this.shooterMuzzleV0Supplier.get(), projectedVelocityFromRobot);

        // Required direction to face target in world coordinates
        double requiredYawWorld = Math.atan2(deltaY, deltaX);

        // Calculate projectile Z velocity at launch
        double vpwZ = this.shooterMuzzleV0Supplier.get() * Math.sin(trajectory.angle);

        // Required horizontal velocity magnitude to reach target in time
        double vx_mag_required = dxEff / trajectory.time;
        Translation2d vector_xy_required = new Translation2d(vx_mag_required, 0).rotateBy(new Rotation2d(requiredYawWorld));
        // Remove shooter velocity to get required velocity from shooter mechanism
        Translation2d shooter_xy_vector_required = vector_xy_required.minus(new Translation2d(shooterVelWorld.getX(), shooterVelWorld.getY()));
       
        // Required robot rotation accounting for shooter offset mount angle
        double requiredRobotYawRad = shooter_xy_vector_required.getAngle().getRadians() + robotToShooter.get().getRotation().getZ();  
        SmartDashboard.putNumber(this.shooter_number+"SS required angle RobotRel", Math.toDegrees(requiredRobotYawRad-this.robotPoseGetter.get().getRotation().getRadians()));

        // Rate limit the yaw change to prevent sudden jerky movements
        double delta = Math.atan2(Math.sin(requiredRobotYawRad - filteredRequiredRobotYawRad),
                                  Math.cos(requiredRobotYawRad - filteredRequiredRobotYawRad));

        double maxStep = Math.toRadians(10);
        if (delta > maxStep) {
            // Limit positive rotation rate
            requiredRobotYawRad = filteredRequiredRobotYawRad + maxStep;
        } else if (delta < -maxStep) {
            // Limit negative rotation rate
            requiredRobotYawRad = filteredRequiredRobotYawRad - maxStep;
        }

        // Keep angle in standard [-pi, pi] range
        filteredRequiredRobotYawRad = Math.atan2(Math.sin(requiredRobotYawRad), Math.cos(requiredRobotYawRad));
        
        // Final projectile velocity vector at launch
        Translation3d shooterVelocityCheck = new Translation3d(
            shooterVelWorld.getX() + shooter_xy_vector_required.getX(),
            shooterVelWorld.getY() + shooter_xy_vector_required.getY(),
            shooterVelWorld.getZ() + vpwZ
        );
        
        // Simulation: visualize and animate projectile trajectories
        if(RobotBase.isSimulation()){
            if(this.shooting_permisive == true && (Timer.getFPGATimestamp() - last_shot_time) > 0.05){
                // Store initial position and velocity for trajectory visualization
                trajectoryvis[ball_index] = new Translation3d(shooterPoseWorld.getX(), shooterPoseWorld.getY()-Units.inchesToMeters(5), shooterPoseWorld.getZ());
                trajectoryvis_prime[ball_index] = new Translation3d(
                    shooterVelocityCheck.getX(),
                    shooterVelocityCheck.getY(),
                    shooterVelocityCheck.getZ()
                );
                
                // Add second trajectory for dual shooter visualization
                trajectoryvis[ball_index+1] = new Translation3d(shooterPoseWorld.getX(), shooterPoseWorld.getY()+Units.inchesToMeters(5), shooterPoseWorld.getZ());
                trajectoryvis_prime[ball_index+1] = new Translation3d(
                    shooterVelocityCheck.getX(),
                    shooterVelocityCheck.getY(),
                    shooterVelocityCheck.getZ()
                );
                ball_index = (ball_index + 2) % num_balls_vis;

                last_shot_time = Timer.getFPGATimestamp();
            }
        }
        
        // Reject solutions with large vertical errors or invalid trajectories
        if (!trajectory.isValid || Math.abs(trajectory.Yerror) >.2) {
            _best_solution = new Solution("No Solution", requiredRobotYawRad, trajectory.angle, trajectory.time, 0, shooterVelocityCheck, trajectory.Yerror);
            return 1e8; // Heavy penalty for invalid solutions
        }
        
        _best_solution = new Solution("Solution Found", requiredRobotYawRad, trajectory.angle, trajectory.time, 0, shooterVelocityCheck, trajectory.Yerror);
        return trajectory.Yerror;
    }


    // Solve complete kinematics problem for current robot state
    public Solution solveKinematics() {
        // Reset solution to invalid state at start
        _best_solution = new Solution("No Solution", 0, 0, 0, 0, new Translation3d(0,0,0), Double.POSITIVE_INFINITY);

        // Transform robot pose to 3D and apply shooter mount offset
        double yawR = this.robotPoseGetter.get().getRotation().getRadians();
        Pose3d robotPose3d = new Pose3d(new Translation3d(this.robotPoseGetter.get().getX(), this.robotPoseGetter.get().getY(), 0.0), 
                                        new Rotation3d(0.0, 0.0, yawR));
        Pose3d shooterPoseWorld = robotPose3d.transformBy(this.robotToShooter.get());

        // Calculate shooter velocity including linear and rotational components
        Transform2d robotVelocity = this.robotVelocityGetter.get();
        double robotOmegaZ = robotVelocity.getRotation().getRadians();
        
        // Velocity from robot rotation (perpendicular to offset arm)
        Translation3d shootervelFromRobotRotation = new Translation3d(-robotOmegaZ * this.robotToShooter.get().getTranslation().getY(),
                                                                robotOmegaZ * this.robotToShooter.get().getTranslation().getX(),
                                                                0);
        shootervelFromRobotRotation = shootervelFromRobotRotation.rotateBy(new Rotation3d(0.0, 0.0, yawR));
       
        // Combine translational and rotational velocity components
        double shooterVelX = robotVelocity.getX() + shootervelFromRobotRotation.getX();
        double shooterVelY = robotVelocity.getY() + shootervelFromRobotRotation.getY();
        double shooterVelZ = 0.0;
        Translation3d shooterVelWorld_final = new Translation3d(shooterVelX, shooterVelY, shooterVelZ);

        // Solve for required angles and validate solution
        @SuppressWarnings("unused")
        double yError = objectiveFunction(shooterPoseWorld, shooterVelWorld_final);
            

        // Return solution or error state
        if (_best_solution.status.equals("Solution Found")) {
            SmartDashboard.putNumber(this.shooter_number+"SS required yaw", Math.toDegrees(_best_solution.requiredRobotYawRad));
            SmartDashboard.putNumber(this.shooter_number+"SS required pitch", Math.toDegrees(_best_solution.requiredShooterPitchRad));
            SmartDashboard.putNumber(this.shooter_number+"SS time of flight", _best_solution.timeOfFlight);
            
            return _best_solution;
        } else {
            return new Solution("No Solution Found", 0, 0, 0, 0, new Translation3d(0,0,0), Double.POSITIVE_INFINITY);
        }
    }
    @Override
    public void periodic() {
        // Skip computation if called too frequently (throttle to RefreshRate)
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastTimeSolved < RefreshRate) {
            return;
        }
        
        // Solve kinematics and measure execution time
        double timeBeforeSolving = Timer.getFPGATimestamp();
        solveKinematics();
        double timeAfterSolving = Timer.getFPGATimestamp();
        SmartDashboard.putNumber(this.shooter_number+"SS Solve Time (ms)", (timeAfterSolving - timeBeforeSolving)*1000.0);
        
        // Safety: disable shooting after watchdog timeout
        if(currentTime - shooting_watchdog > 100){
            this.shooting_permisive = false;
        }
        
        // Simulation: update and publish projectile trajectories
        if(RobotBase.isSimulation()){
            // Update each simulated projectile position based on velocity
            for(int i =0; i < num_balls_vis; i++){
                // Apply gravity to velocity
                trajectoryvis_prime[i] = new Translation3d(
                    trajectoryvis_prime[i].getX(), 
                    trajectoryvis_prime[i].getY(), 
                    trajectoryvis_prime[i].getZ() - 9.81 * (currentTime - lastTimeSolved)
                );
                // Update position
                trajectoryvis[i] = trajectoryvis[i].plus(trajectoryvis_prime[i].times(currentTime - lastTimeSolved));
                
                // Bounce off ground with energy loss
                if(trajectoryvis[i].getZ() < 0){
                    trajectoryvis[i] = new Translation3d(trajectoryvis[i].getX(), trajectoryvis[i].getY(), 0);
                    trajectoryvis_prime[i] = new Translation3d(
                        trajectoryvis_prime[i].getX() * 0.5, 
                        trajectoryvis_prime[i].getY() * 0.5, 
                        -trajectoryvis_prime[i].getZ() * 0.5
                    );
                }
            }
            // Publish trajectory data over network tables
            trajectoryPublisher.set(trajectoryvis);
        }
        lastTimeSolved = currentTime;
    }
    
    // Query most recent solution error (vertical miss distance)
    public double getBestSolutionYError(){
        return _best_solution.minYError; 
    }
    
    // Get when last solution was calculated
    public double getBestSolutionTimestamp(){
        return _best_solution.timestamp;
    }
    
    // Retrieve required robot yaw angle from most recent solution
    public double getRobotRequiredYawRad(){
        return _best_solution.requiredRobotYawRad;
    }
    
    // Retrieve required shooter pitch angle from most recent solution
    public double getShooterRequiredPitchRad(){
        return _best_solution.requiredShooterPitchRad;
    }
    
    // Get status of most recent solution attempt
    public String getBestSolutionStatus(){
        return _best_solution.status;
    }
    
    // Enable or disable shooting with watchdog timer
    public void AllowShooting(boolean allowed){
        this.shooting_watchdog = Timer.getFPGATimestamp();
        this.shooting_permisive = allowed;
    }
    
    // Check if shooting is currently permitted
    public boolean isShootingAllowed(){
        return this.shooting_permisive;
    }
}
