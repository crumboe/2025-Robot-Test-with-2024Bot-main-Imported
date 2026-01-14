package frc.robot.subsystems.Locomotion;

/*
 * ╔════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  DRIVE SUBSYSTEM: Swerve Kinematics, Pose Estimation, and Sensor Fusion                                   ║
 * ╚════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
 * 
 * PURPOSE:
 * --------
 * This is the core drivetrain subsystem managing the four-wheel swerve drive. It handles:
 *   • Input processing: converts velocity commands (forward, strafe, omega) to swerve module speeds/angles
 *   • Holonomic kinematics: independent control of translational and rotational motion
 *   • Pose estimation: maintains a SwerveDrivePoseEstimator (Extended Kalman Filter) for robot localization
 *   • Sensor fusion: combines odometry (motor encoders, gyro) with vision (AprilTag camera) measurements
 *   • Current limiting: scales accelerations to prevent breaker trips
 *   • Autonomous integration: PathPlanner command scheduling with custom trajectory following
 * 
 * SWERVE DRIVE PRINCIPLES:
 * -----------------------
 * A swerve drive has 4 modules (one per corner), each with:
 *   • A drive motor (propels the wheel forward/backward)
 *   • A steer motor (rotates the wheel to point in any direction)
 * 
 * This allows the robot to:
 *   • Move in any direction simultaneously (omniirectional motion)
 *   • Rotate in place WITHOUT stalling (unlike differential drive)
 *   • Strafe sideways for tactical positioning
 * 
 * KINEMATICS CALCULATION (processInput method):
 * -----------------------------------------------
 * Input: operator joystick commands
 *   • forward:  desired velocity in +Y direction (m/s)
 *   • strafe:   desired velocity in +X direction (m/s)
 *   • omega:    desired angular velocity (rad/s)
 * 
 * Output: target speeds and angles for each module
 * 
 * Physics: Treating the robot as a rigid body with rotation about its center:
 * 
 *   For a module at position (dx, dy) relative to robot center:
 *   
 *     v_wheel_x = forward - omega * dy     [forward component - rotational effect]
 *     v_wheel_y = strafe  + omega * dx     [strafe component + rotational effect]
 *     
 *     magnitude = sqrt(v_wheel_x² + v_wheel_y²)  [wheel speed in m/s]
 *     angle     = atan2(v_wheel_y, v_wheel_x)    [wheel direction in radians]
 * 
 * This code uses precomputed half-wheelbase and half-trackwidth to avoid redundant calculations:
 *   • omegaL2 = omega * (wheelbase / 2)      [contribution to front/back modules]
 *   • omegaW2 = omega * (trackwidth / 2)     [contribution to left/right modules]
 * 
 * Then applies these to each corner:
 *   • Front-Left:  v = sqrt((strafe + omegaL2)² + (forward - omegaW2)²)
 *   • Front-Right: v = sqrt((strafe + omegaL2)² + (forward + omegaW2)²)
 *   • Back-Left:   v = sqrt((strafe - omegaL2)² + (forward - omegaW2)²)
 *   • Back-Right:  v = sqrt((strafe - omegaL2)² + (forward + omegaW2)²)
 * 
 * POSE ESTIMATION AND SENSOR FUSION:
 * -----------------------------------
 * The SwerveDrivePoseEstimator fuses three types of measurements:
 * 
 *   1. PREDICTION (Odometry):
 *      - Motor encoder positions → swerve module positions
 *      - Kinematics calculation → chassis speeds
 *      - Gyro rotation → heading angle
 *      - Runs at 50 Hz (every 20 ms) in periodic()
 * 
 *   2. CORRECTION (Vision):
 *      - AprilTag detections from Camera subsystem
 *      - Absolute field position via addVisionMeasurement()
 *      - Runs asynchronously when Camera publishes measurements
 *      - Standard deviations control how much vision affects pose
 * 
 *   3. HEADING:
 *      - Gyro provides rotation measurement with low noise
 *      - Provides constraint for Extended Kalman Filter
 * 
 * Extended Kalman Filter Equations:
 * 
 *   PREDICTION STEP (periodic):
 *     x̂⁻(k+1) = f(x̂(k), u(k))          [predict pose from odometry]
 *     P⁻(k+1)  = F·P(k)·Fᵀ + Q          [predict covariance, Q = process noise]
 * 
 *   MEASUREMENT UPDATE (from Camera.periodic() → addVisionMeasurement):
 *     y(k)     = z(k) - h(x̂⁻(k))        [innovation: vision vs prediction]
 *     S(k)     = H·P⁻(k)·Hᵀ + R          [innovation covariance, R = measurement noise]
 *     K(k)     = P⁻(k)·Hᵀ / S(k)        [Kalman gain (depends on R = camera stddev)]
 *     x̂(k)     = x̂⁻(k) + K(k)·y(k)      [correct prediction with vision measurement]
 *     P(k)     = (I - K(k)·H) · P⁻(k)   [reduce uncertainty after measurement]
 * 
 * The Camera subsystem reports measurement uncertainty as a standard deviation matrix:
 *   • Single tag: stddev = [2.0, 2.0, 4.0] → low confidence, small correction
 *   • Multi tag:  stddev = [0.5, 0.5, 1.0] → high confidence, large correction
 * 
 * ANGLE COMMAND TRACKING:
 * -----------------------
 * The robot maintains a target heading angle (angleCommand) that can be:
 *   • Updated continuously: angleCommand += omega * dt  (follows operator rotation)
 *   • Fixed target: angleCommand = constant            (maintains heading)
 *   • Overridden: angleCommand = shooterSolverYaw      (auto-aim at target)
 * 
 * P-control proportionally rotates the robot toward the target:
 *     omega_out = (angleCommand - current_heading) * Kp    [Kp = 4 rad/s per radian error]
 * 
 * This decouples rotational control from forward/strafe, allowing the robot to strafe while
 * maintaining a specific heading (useful for shooter alignment).
 * 
 * CURRENT LIMITING:
 * -----------------
 * To prevent battery voltage sag and breaker trips:
 *   1. Calculate motor current draw based on acceleration and wheel torque constant
 *   2. If total current exceeds 170 A, scale down all acceleration commands
 *   3. Recompute module speeds with reduced acceleration to stay under limit
 *   4. Maintain module angles unchanged
 * 
 * This preserves motion direction while limiting power draw.
 * 
 * INTEGRATION WITH AUTONOMY:
 * --------------------------
 * PathPlanner autonomous paths are executed via FollowPathCommand103:
 *   1. FollowPathCommand calls driveFieldRelativeAuto() every cycle
 *   2. PathPlanner provides target chassis speeds in field coordinates
 *   3. driveFieldRelativeAuto() converts field speeds → robot speeds
 *   4. processInput() converts robot speeds → module speeds/angles
 *   5. Modules execute, Drive.periodic() samples encoders
 *   6. SwerveDrivePoseEstimator updates pose estimate
 * 
 * During autonomous, vision measurements continue to correct odometry drift.
 * 
 * KEY STATE VARIABLES:
 * --------------------
 *   • robotPoseEstimate: SwerveDrivePoseEstimator maintaining (x, y, theta)
 *   • angleCommand:      Target robot heading in radians
 *   • {speed,angle}{FL,FR,BL,BR}: Module speeds (m/s) and angles (degrees)
 *   • lastSpeed*:        Previous speeds used for acceleration calculation
 *   • {current,force}*:  Motor current/force for limiting
 * 
 * SIMULATION VS HARDWARE:
 * -----------------------
 * In simulation (RobotBase.isSimulation()):
 *   • Uses angleCommand as gyro rotation (operator controls heading directly)
 *   • Uses simulated module positions (not real encoders)
 *   • Pose estimator initialized with low odometry noise (0.05 m, 5°)
 * 
 * On hardware:
 *   • Uses real gyro rotation from IMU
 *   • Uses real encoder positions from SPARK MAX motors
 *   • Pose estimator initialized with same odometry noise assumptions
 */

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.FrameConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.commands.TunableHolonomicController;
import frc.robot.commands.flowerTest;
import frc.robot.subsystems.ultilities.FollowPathCommand103;
import frc.robot.subsystems.ultilities.Gyros.Gyro;
import edu.wpi.first.math.util.Units;

import java.util.Optional;
// import edu.wpi.first.units.measure.LinearVelocity;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;

import java.util.Arrays;
import java.util.Collections;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import java.util.function.Supplier;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Drive extends SubsystemBase {
    // Singleton instance for drive subsystem
    private static Drive instance;

    // Maximum achievable robot velocity
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    // Maximum achievable robot angular velocity
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    // Robot configuration loaded from Path Planner
    public RobotConfig robotConfig;

    // Swerve module instances for each corner
    private static SwerveModule frontLeft;
    private static SwerveModule backLeft;
    private static SwerveModule frontRight;
    private static SwerveModule backRight;

    // Physical robot dimensions for kinematics
    public static final double length_with_bumpers = .9271;
    // Default starting position for autonomous
    public static final Pose2d start_pose_defualt = new Pose2d(1,1, Rotation2d.fromDegrees(0));

    // Motor inversion flags for swerve module configuration
    private final boolean invertDrive = false;
    private final boolean invertSteer = true;
    
    // Gyro sensor for rotation measurement
    private Gyro _gyro;
    // Supplier for shooter solver yaw requirement
    private Supplier<Double> ShooterSolverYawSupplier;
    // Supplier for shooter solver solution validity
    private Supplier<Boolean> ShooterSolverSolutionFoundSupplier;
    // Flag indicating shooter solver is providing valid commands
    private boolean _driveCorrect = false;

    // Pose estimation using vision and odometry
    private SwerveDrivePoseEstimator robotPoseEstimate;
    // Target robot heading angle in radians
    private double angleCommand;
    // Flag indicating angle command needs initialization
    public boolean angleCommandNotSet = true;
    // Previous angle error for derivative calculation
    private double lastAngleError = 0;
    // PID tuning utility reference
    public flowerTest PID_TUNING;
    
    // Previous module speeds for acceleration calculation
    private double lastSpeedFL, lastSpeedFR, lastSpeedBL, lastSpeedBR;

    // Intermediate kinematics calculations
    private double angleErr;
    @SuppressWarnings("unused")
    private double DangleErr;
    // Rotational velocity output to modules
    private double omega_out;
    // Half wheelbase times angular velocity
    private double omegaL2;
    // Half track width times angular velocity
    private double omegaW2;
    // Kinematics constants A through D for module velocity calculations
    private double A;
    private double B;
    private double C;
    private double D;
    
    // Module drive speeds (m/s)
    private double speedFL;
    private double speedBL;
    private double speedFR;
    private double speedBR;
    
    // Module speed deltas for acceleration calculation
    private double deltaFL;
    private double deltaFR;
    private double deltaBL;
    private double deltaBR;
    
    // Module accelerations (m/s²)
    private double accelFL;
    private double accelFR;
    private double accelBL;
    private double accelBR;
    
    // Calculated forces on each drive motor
    private double forceFL;
    private double forceFR;
    private double forceBL;
    private double forceBR;
    
    // Estimated motor current draw (amps)
    private double currentFL;
    private double currentFR;
    private double currentBL;
    private double currentBR;
    // Total current draw across all motors
    private double totalCurrent;
    // Current limit scaling factor
    private double currentRatio;
    
    // Module steer angles (degrees)
    private double angleFL;
    private double angleBL;
    private double angleFR;
    private double angleBR;
    
    // Maximum speed among all modules
    private double maxSpeed;
    @SuppressWarnings("unused")
    private double lastInputTime = 0;

    // Factory method for singleton access
    public static Drive getInstance(Gyro gyro) {
        if (instance == null) {
            instance = new Drive(gyro);
        }
        return instance;
    }
    
    // Constructor initializes swerve modules and pose estimation
    private Drive(Gyro gyro) {
        this._gyro = gyro;

        // Create swerve modules for each corner of the chassis
        frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive, invertSteer);
        frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive, invertSteer);
        backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive, invertSteer);
        backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive, invertSteer);
        
        // Initialize pose estimator with appropriate standard deviations
        if(RobotBase.isSimulation()){
            // Simulation uses commanded angle and module positions
            robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics,
                    this.getSimRotation2d(), getPositions(), start_pose_defualt,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        }else{
            // Hardware uses gyro rotation
            robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics,
                    this._gyro.getGyroRotation2D(), getPositions(), start_pose_defualt,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        }

        try {
            // Load robot configuration from PathPlanner settings
            robotConfig = RobotConfig.fromGUISettings();
            
            // Initialize PID values from persistent storage or defaults
            double seedTP = Preferences.getDouble("PID/transP", 5);
            double seedTI = Preferences.getDouble("PID/transI", 0);
            double seedTD = Preferences.getDouble("PID/transD", 0);
        
            double seedRP = Preferences.getDouble("PID/rotP", 5);
            double seedRI = Preferences.getDouble("PID/rotI", 0);
            double seedRD = Preferences.getDouble("PID/rotD", 0);
            var ctrl = Optional.of(new TunableHolonomicController(
                seedTP, seedTI, seedTD,
                seedTP, seedTI, seedTD,
                seedRP, seedRI, seedRD,
                Math.toRadians(720), Math.toRadians(720)));

            // Configure PathPlanner for field-centric autonomous
            AutoBuilder.configureCustom(
                (path)->this.PathCommandBuilder103(path,(ctrl)), // Custom path command with pose-based error checking
                this::getPose,
                this::resetPose,
                () -> {
                    // Mirror paths for red alliance
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                true
            );
            
            // Initialize angle command to current heading
            angleCommand = getPose().getRotation().getRadians();

        } catch (Exception e) {
            e.printStackTrace();
        }
        angleCommandNotSet = true;
    }
    /**
     * Periodic update - runs once per scheduler cycle (20 ms)
     */
    @Override()
    public void periodic() {
        // Update pose estimate from odometry and gyro
        if (!RobotBase.isSimulation()) {
            // Hardware: use real gyro rotation
            robotPoseEstimate.update(this._gyro.getRotation2d(), getPositions());
        } else {
            // Simulation: use commanded angle and simulated positions
            Rotation2d simangle = new Rotation2d(angleCommand);
            var sim_positions = new SwerveModulePosition[] {
                frontLeft.getSimPosition(),
                frontRight.getSimPosition(),
                backLeft.getSimPosition(),
                backRight.getSimPosition()
            };
            robotPoseEstimate.update(simangle, sim_positions);
        }

        // Publish pose data to PathPlanner and SmartDashboard
        PPLibTelemetry.setCurrentPose(getPose());
        Pose2d RobotPose = robotPoseEstimate.getEstimatedPosition();
        SmartDashboard.putNumber("x_field", RobotPose.getX());
        SmartDashboard.putNumber("y_field", RobotPose.getY());
        SmartDashboard.putNumber("theta_field", RobotPose.getRotation().getDegrees());
    }

    // Convert field-relative speeds to module commands for autonomous
    public void driveFieldRelativeAuto(ChassisSpeeds fieldRelativeSpeeds, DriveFeedforwards feedforwards) {
        double xPositive = fieldRelativeSpeeds.vxMetersPerSecond;
        double yPositive = fieldRelativeSpeeds.vyMetersPerSecond;
        
        // Create target velocity vector in field coordinates
        Translation2d FieldVelocity = new Translation2d(xPositive, yPositive);
        // Rotate to robot-centric coordinates
        Translation2d RobotVelocity = FieldVelocity.rotateBy(getPose().getRotation().unaryMinus());

        double forward = RobotVelocity.getX();
        double strafe = RobotVelocity.getY();
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        
        // Process input to drive modules
        this.processInput(forward, strafe, omega, false, true, false);
    }
    
    // Core input processing: convert command values to swerve module states
    public void processInput(double forward, double strafe, double omega, boolean deadStick, boolean directOmega, boolean useAngleCommandForOmega) {
        double dt = 0.02;
    
        // Reset input if dead stick engaged
        if(deadStick){
            forward = 0;
            strafe = 0;
            this.angleCommand = getPose().getRotation().getRadians();
        }
        
        this._driveCorrect = false;
        
        // Calculate angle command from angular velocity or direct angle
        if(useAngleCommandForOmega){
            this.angleCommand = omega;
        }else{
            this.angleCommand = MathUtil.angleModulus(this.angleCommand + 0.02*omega*0.6);
        }
        
        // Override angle command with shooter solver if available
        if(this.ShooterSolverSolutionFoundSupplier.get()){
            double solverYaw = this.ShooterSolverYawSupplier.get();
            if(!Double.isNaN(solverYaw)){
                this.angleCommand = solverYaw;
                this._driveCorrect = true;
                directOmega = false;
            }
        }
        SmartDashboard.putNumber("angle Command", angleCommand/Math.PI*180);

        // Calculate rotational error and apply proportional control
        angleErr = MathUtil.angleModulus(this.angleCommand - this.getPose().getRotation().getRadians());
        DangleErr = angleErr - lastAngleError;
        SmartDashboard.putNumber("angle error", angleErr/Math.PI*180);
        
        if(directOmega){
            omega_out = omega;
        }else{
            // P control for angle: gain of 4 rad/s per radian error
            omega_out = angleErr * 4;
        }
        
        SmartDashboard.putNumber("omegaOut", omega_out);
        
        // Calculate kinematics values for each module
        omegaL2 = omega_out * (DriveConstants.FrameConstants.kWheelBase / 2.0);
        omegaW2 = omega_out * (DriveConstants.FrameConstants.kTrackWidth / 2.0);

        // Intermediate kinematics calculations
        A = strafe - omegaL2;
        B = strafe + omegaL2;
        C = forward - omegaW2;
        D = forward + omegaW2;

        // Calculate required module drive speeds from kinematics
        speedFL = speed(B, C);
        speedBL = speed(A, C);
        speedFR = speed(B, D);
        speedBR = speed(A, D);

        // Calculate speed changes for current limiting
        deltaFL = speedFL - lastSpeedFL;
        deltaFR = speedFR - lastSpeedFR;
        deltaBL = speedBL - lastSpeedBL;
        deltaBR = speedBR - lastSpeedBR;

        // Calculate accelerations from speed deltas
        accelFL = Math.abs(deltaFL) / dt;
        accelFR = Math.abs(deltaFR) / dt;
        accelBL = Math.abs(deltaBL) / dt;
        accelBR = Math.abs(deltaBR) / dt;

        // Estimate motor forces from accelerations
        forceFL = 60 * accelFL / 4; // Divide by 4 for four wheels
        forceFR = 60 * accelFR / 4;
        forceBL = 60 * accelBL / 4;
        forceBR = 60 * accelBR / 4;

        // Calculate current draw for each motor
        currentFL = (forceFL * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentFR = (forceFR * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentBL = (forceBL * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentBR = (forceBR * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;

        // Apply current limiting by scaling acceleration
        totalCurrent = currentFL + currentFR + currentBL + currentBR;
        if(totalCurrent > 170 && totalCurrent > 0){
            currentRatio = 170 / totalCurrent; // Scale down accelerations
            deltaFL *= currentRatio;
            deltaFR *= currentRatio;
            deltaBL *= currentRatio;
            deltaBR *= currentRatio;

            speedFL = lastSpeedFL + deltaFL;
            speedFR = lastSpeedFR + deltaFR;
            speedBL = lastSpeedBL + deltaBL;
            speedBR = lastSpeedBR + deltaBR;
        }
        
        // Calculate required module steer angles from kinematics
        angleFL = angle(B, C);
        angleBL = angle(A, C);
        angleFR = angle(B, D);
        angleBR = angle(A, D);

        // Find maximum speed and normalize all speeds if needed
        maxSpeed = Collections.max(Arrays.asList(Math.abs(speedFL), Math.abs(speedBL), Math.abs(speedFR), Math.abs(speedBR)));
        if(maxSpeed > DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond){
            double speedratio = maxSpeed / DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
            speedFL = speedFL / speedratio;
            speedFR = speedFR / speedratio;
            speedBL = speedBL / speedratio;
            speedBR = speedBR / speedratio;
        }

        // Apply commands to modules or stop if dead stick
        if (deadStick) {
            frontLeft.setDriveSpeed(0);
            frontRight.setDriveSpeed(0);
            backLeft.setDriveSpeed(0);
            backRight.setDriveSpeed(0);

            frontLeft.setSteerSpeed(0);
            frontRight.setSteerSpeed(0);
            backLeft.setSteerSpeed(0);
            backRight.setSteerSpeed(0);

        } else {
            // Send angles and speeds to each swerve module
            frontLeft.setSwerve(angleFL, speedFL, this._driveCorrect);
            frontRight.setSwerve(angleFR, speedFR, this._driveCorrect);
            backLeft.setSwerve(angleBL, speedBL, this._driveCorrect);
            backRight.setSwerve(angleBR, speedBR, this._driveCorrect);
        }
        
        // Store speeds for next cycle's acceleration calculation
        lastSpeedFL = speedFL;
        lastSpeedFR = speedFR;
        lastSpeedBL = speedBL;
        lastSpeedBR = speedBR;

        lastInputTime = Timer.getFPGATimestamp();
    }
    
    // Helper: calculate speed magnitude from two velocity components
    private double speed(double val1, double val2) {
        return Math.hypot(val1, val2);
    }

    // Helper: calculate angle (degrees) from two velocity components
    private double angle(double val1, double val2) {
        return Math.toDegrees(Math.atan2(val1, val2));
    }

    // Send vision measurement to pose estimator with optional confidence weighting
    public void addVisionMeasurement(Pose2d poseEst, double timestamp) {
        robotPoseEstimate.addVisionMeasurement(poseEst, timestamp);
    }

    public void addVisionMeasurement(Pose2d poseEst, double timestamp, Matrix<N3, N1> stdDevs) {
        robotPoseEstimate.addVisionMeasurement(poseEst, timestamp, stdDevs);    
    }

    // Stop all swerve modules (lock steering, zero drive)
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set all modules to coast or brake mode
    public void setDrivesMode(IdleMode idleMode) {
        frontLeft.setDriveMode(idleMode);
        frontRight.setDriveMode(idleMode);
        backLeft.setDriveMode(idleMode);
        backRight.setDriveMode(idleMode);
    }

    public void setDriveModeCoast() {
        setDrivesMode(IdleMode.kCoast);
        isCoastMode = true;
    }

    public void setDriveModeBrake() {
        setDrivesMode(IdleMode.kBrake);
        isCoastMode = false;
    }

    // Disable acceleration ramping for sharp response
    public void disableRamping() {
        frontLeft.driveMotorRamp(false);
        frontRight.driveMotorRamp(false);
        backLeft.driveMotorRamp(false);
        backRight.driveMotorRamp(false);
    }

    // Get current state of all swerve modules
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        return states;
    }

    private boolean isCoastMode = false;

    // Check current idle mode state
    public boolean toggleMode() {
        return isCoastMode;
    }
    public void addPIDTUNING(flowerTest PIDTUN){
        this.PID_TUNING = PIDTUN;
    }
    public void startPIDTuning(boolean run){
        PID_TUNING.setRunning(run);
        if(run){
            CommandScheduler.getInstance().schedule(PID_TUNING);
        }
    }
    // Link shooter solver to drive for auto-aiming
    public void setup_shooter_solver_link(Supplier<Double> ShooterSolverYawSupplier,
        Supplier<Boolean> ShooterSolverSolutionFoundSupplier){
        this.ShooterSolverYawSupplier = ShooterSolverYawSupplier;
        this.ShooterSolverSolutionFoundSupplier = ShooterSolverSolutionFoundSupplier;
    }

    // Reset estimated robot pose to known value
    public void resetPose(Pose2d pose) {
        System.out.println("\n\n\nResetting Pose to: " + pose.toString());
        if(RobotBase.isSimulation()){
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
            robotPoseEstimate.resetRotation(pose.getRotation());
            this.angleCommand = pose.getRotation().getRadians();
            robotPoseEstimate.resetPosition(pose.getRotation(), getPositions(), pose);
            
            System.out.println("Resetting Pose to: " + robotPoseEstimate.getEstimatedPosition().toString());
        }else{
            robotPoseEstimate.resetPosition(_gyro.getRotation2d(), getPositions(), pose);
        }
    }

    // Reset robot heading to zero degrees
    public void resetPoseRotation() {
        System.out.println("\n\n\nResetting angle");
        robotPoseEstimate.resetRotation(Rotation2d.fromDegrees(0));
    }
    
    // Get simulated rotation for pose estimation in sim
    public Rotation2d getSimRotation2d(){
        return new Rotation2d(angleCommand);
    }
    
    // Get current estimated robot pose
    public Pose2d getPose() {
        return robotPoseEstimate.getEstimatedPosition();
    }
    
    // Publish steer encoder values to SmartDashboard for debugging
    public void getSteerEncoderVal() {
        SmartDashboard.putNumber("angleLF", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("angleRF", frontRight.getTurningPosition());
        SmartDashboard.putNumber("angleLB", backLeft.getTurningPosition());
        SmartDashboard.putNumber("angleRB", backRight.getTurningPosition());
    }

    // Get position of each swerve module (used for odometry)
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
    
    // Get chassis speeds in robot-relative coordinates
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    // Get chassis speeds in field-relative coordinates
    public ChassisSpeeds getSpeedsWorld() {
        var RobotSpeed = DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        Translation2d RobotVelocity = new Translation2d(RobotSpeed.vxMetersPerSecond, RobotSpeed.vyMetersPerSecond);
        Translation2d FieldVelocity = RobotVelocity.rotateBy(getPose().getRotation());
        return new ChassisSpeeds(
            FieldVelocity.getX(),
            FieldVelocity.getY(),
            RobotSpeed.omegaRadiansPerSecond
        );
    }
    public Transform2d getSpeedsWorld(boolean returnTransform) {
        ChassisSpeeds RobotSpeed = getSpeedsWorld();
        return new Transform2d(
            new Translation2d(RobotSpeed.vxMetersPerSecond, RobotSpeed.vyMetersPerSecond),
            new Rotation2d(RobotSpeed.omegaRadiansPerSecond)
        );
    }
    
    // Build PathPlanner command with tunable controllers
    public FollowPathCommand103 PathCommandBuilder103(PathPlannerPath path, Optional<TunableHolonomicController> inputCtrl){
        TunableHolonomicController ctrl = new TunableHolonomicController(
                5.0, 0.0, 0.0,   // x PID
                5.0, 0.0, 0.0,   // y PID
                5.0, 0.0, 0.0,   // theta PID
                FrameConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, 
                FrameConstants.kphysicalMaxAngularAccelerationRadiansPerSecond);
        if( inputCtrl.isPresent()){
            ctrl = inputCtrl.get();
        }
        
        return new FollowPathCommand103(
            path,
            this::getPose,
            this::getSpeedsWorld,
            this::driveFieldRelativeAuto,
            ctrl,
            this.robotConfig,
            () -> {
                // Mirror paths for red alliance
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
            );
    }
}