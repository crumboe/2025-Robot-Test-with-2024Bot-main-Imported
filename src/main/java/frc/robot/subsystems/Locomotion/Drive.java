package frc.robot.subsystems.Locomotion;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.FrameConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.commands.TunableHolonomicController;
import frc.robot.commands.flowerTest;
import frc.robot.subsystems.ultilities.FollowPathCommand103;
import frc.robot.subsystems.ultilities.Gyros.Gyro;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
// import edu.wpi.first.units.measure.LinearVelocity;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;

import java.util.Arrays;
import java.util.Collections;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import java.util.function.Supplier;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Drive extends SubsystemBase {

    private static Drive instance;

    // private SwerveSetpointGenerator setPointgenerator; // team 254's antislippage
    // private SwerveSetpoint previousSetpoint; // team 254's antislippage
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public RobotConfig robotConfig;

    private static SwerveModule frontLeft;
    private static SwerveModule backLeft;
    private static SwerveModule frontRight;
    private static SwerveModule backRight;

    public static final double length_with_bumpers = .9271;
    public static final Pose2d start_pose_defualt = new Pose2d(1,1,
            Rotation2d.fromDegrees(0));

    private final boolean invertDrive = false;// true;
    private final boolean invertSteer = true; // true;
    private Gyro _gyro;
    private Supplier<Double> ShooterSolverYawSupplier;
    private Supplier<Boolean> ShooterSolverSolutionFoundSupplier;
    private boolean _driveCorrect = false;


    private SwerveDrivePoseEstimator robotPoseEstimate;
    private Transform2d robotVelEstimate;
    private Pose2d robotvelEsitmateLastPose;
    private double robotVelEstimateLastTime;
    private double angleCommand;
    public boolean angleCommandNotSet = true;
    private double lastAngleError=0;
    public flowerTest PID_TUNING;
    private double lastSpeedFL ,lastSpeedFR ,lastSpeedBL ,lastSpeedBR ;


    private double angleErr;
    private double DangleErr;
    private double omega_out;
    private double omegaL2;
    private double omegaW2;
    private double A;
    private double B;
    private double C;
    private double D;
    private double speedFL;
    private double speedBL;
    private double speedFR;
    private double speedBR;
    private double deltaFL;
    private double deltaFR;
    private double deltaBL;
    private double deltaBR;
    private double accelFL;
    private double accelFR;
    private double accelBL;
    private double accelBR;
    private double forceFL;
    private double forceFR;
    private double forceBL;
    private double forceBR;
    private double currentFL;
    private double currentFR;
    private double currentBL;
    private double currentBR;
    private double totalCurrent;
    private double currentRatio;
    private double angleFL;
    private double angleBL;
    private double angleFR;
    private double angleBR;
    private double maxSpeed;
    private double lastInputTime=0;

    /*
     * Set up the drive by passing in the gyro and then configuring the individual
     * swerve modules.
     * Note the order that the modules are in. Be consistant with the order in the
     * odometry.
     */
    private Drive(Gyro gyro) {
        
        this._gyro = gyro;


        frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive,
                invertSteer);

        frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive,
                invertSteer);

        backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive,
                invertSteer);

        backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive,
                invertSteer);
        if(RobotBase.isSimulation()){
            robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics,
                    this.getSimRotation2d(), getPositions(), start_pose_defualt,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        }else{
            robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics,
                    this._gyro.getGyroRotation2D(), getPositions(), start_pose_defualt,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        }
        robotVelEstimate = new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
        robotvelEsitmateLastPose = getPose();
        robotVelEstimateLastTime = Timer.getFPGATimestamp();
        try {
            robotConfig = RobotConfig.fromGUISettings();
            // Initialize PID values on SmartDashboard for tuning
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

           
            // TODO : Choose which Autobuilder setup makes the most sense, remove other functions referencing AutoBuilder
            AutoBuilder.configureCustom(
                // this::PathCommandBuilder,  // original path command builder using robot centric
                (path)->this.PathCommandBuilder103(path,(ctrl)), // field centric path command builder with pose based error checking
                this::getPose,
                this::resetPose,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                true

            );
            angleCommand = getPose().getRotation().getRadians();
            // configureAutoBuilderDeprec();

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();

        }
        angleCommandNotSet = true;
    }
    // keeping this in case new autobuilder doesn't work 11\7\25
    //FIXME : Remove after testing new AutoBuilder
    public void configureAutoBuilderDeprec(){
        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                this::driveRobotRelative,
                DriveConstants.pathFollowerConfig,
                robotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }
    // FIXME : Remove after testing new AutoBuilder
    public Command PathCommandBuilder(PathPlannerPath path){
        
        
        // System.out.printf("%f %f %f\n",p,i,d);
        
        return new FollowPathCommand(
            path,
            this::getPose,
            this::getSpeeds,
            this::driveRobotRelativeFF,
            Constants.DriveConstants.pathFollowerConfig,
            this.robotConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
            );

    }
    // FIXME : Test this new field centric path command builder with error checking and smart dashboard PIDs

    public FollowPathCommand103 PathCommandBuilder103(PathPlannerPath path, Optional<TunableHolonomicController> inputCtrl){
        
        
        TunableHolonomicController ctrl = new TunableHolonomicController(
                5.0, 0.0, 0.0,   // x PID
                5.0, 0.0, 0.0,   // y PID
                5.0, 0.0, 0.0,   // theta PID
                FrameConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, FrameConstants.kphysicalMaxAngularAccelerationRadiansPerSecond);
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
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
            );

    }
    // Public Methods
    public Transform2d getVelocityWorld() {
        if(RobotBase.isSimulation()){
            SwerveModuleState[] states = new SwerveModuleState[4];
            states[0] = frontLeft. getSimState();
            states[1] = frontRight.getSimState();
            states[2] = backLeft.  getSimState();
            states[3] = backRight. getSimState();
            var ChassisSpeeds = DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(states);
            Transform2d robotVelEstimateSim = new Transform2d(
                new Translation2d(ChassisSpeeds.vxMetersPerSecond, ChassisSpeeds.vyMetersPerSecond).rotateBy(getSimRotation2d()),
                new Rotation2d(0)
            );
            return robotVelEstimateSim;
        }else{
            return robotVelEstimate;
        }
    }
    public Pose2d getPose() {
        return robotPoseEstimate.getEstimatedPosition();
    }

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

    public void resetPoseRotation() {
        System.out.println("\n\n\nResetting angle");
        robotPoseEstimate.resetRotation(Rotation2d.fromDegrees(0));
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    public ChassisSpeeds getSpeedsWorld() {
        var RobotSpeed = DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        Translation2d RobotVelocity = new Translation2d(RobotSpeed.vxMetersPerSecond, RobotSpeed.vyMetersPerSecond);
        Translation2d FieldVelocity = RobotVelocity.rotateBy(getPose().getRotation()); // Rotate // our // target // vector // around // the // robot's // center, // opposite // to // the // robots // heading
        return new ChassisSpeeds(
            FieldVelocity.getX(),
            FieldVelocity.getY(),
            RobotSpeed.omegaRadiansPerSecond
        );
    }
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, this.getPose().getRotation()));
    }
    
    public void driveRobotRelativeFF(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
        driveRobotRelative(robotRelativeSpeeds);
    }
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // SmartDashboard.putNumber("driveRRSpeedx", robotRelativeSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("driveRRSpeedy", robotRelativeSpeeds.vyMetersPerSecond);
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = DriveConstants.FrameConstants.kDriveKinematics
                .toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);

        // Trying this from Cheesypoofs... Looks pretty heavy code wise, so may need to
        // be cut for rio compute
        // previousSetpoint = setPointgenerator.generateSetpoint(
        // previousSetpoint, // The previous setpoint
        // robotRelativeSpeeds, // The desired target speeds
        // 0.02 // The loop time of the robot code, in seconds
        // );
        // setModuleStates(previousSetpoint.moduleStates());

    }

    

    public static Drive getInstance(Gyro gyro) {
        if (instance == null) {
            instance = new Drive(gyro);
        }
        return instance;
    }
    public void driveFieldRelativeAuto(ChassisSpeeds fieldRelativeSpeeds,DriveFeedforwards feedforwards) {
        double xPositive = fieldRelativeSpeeds.vxMetersPerSecond;
        double yPositive = fieldRelativeSpeeds.vyMetersPerSecond;
        
        Translation2d FieldVelocity = new Translation2d(xPositive, yPositive); // create the target vector in space
                                                                          // X_field,
        Translation2d RobotVelocity = FieldVelocity.rotateBy(getPose().getRotation().unaryMinus()); // Rotate // our // target // vector // around // the // robot's // center, // opposite // to // the // robots // heading
        // New vector should be in Space X_robot, Y_Robot

        double forward = RobotVelocity.getX();
        double strafe = RobotVelocity.getY();
        
        
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        
        this.processInput(forward, strafe, omega, false,true,false);
        
    }
    
    
    
    public void processInput(double forward, double strafe, double omega, boolean deadStick,boolean directOmega,boolean useAngleCommandForOmega) {
        double dt = 0.02;
        // if(deadStick){ // if there is a dead stick, make the command angle match the actual angle
        //     this.angleCommand = getPose().getRotation().getRadians();
        //     angleCommandNotSet = false;
        // }
        if(deadStick){
            forward = 0;
            strafe = 0;
            this.angleCommand = getPose().getRotation().getRadians();
        }
        this._driveCorrect = false;
        // TODO: 0.6  multiplier to reduce overall input of angle command to reduce overshooting.
        if(useAngleCommandForOmega){
            this.angleCommand = omega;
        }else{
            this.angleCommand = MathUtil.angleModulus(this.angleCommand + 0.02*omega*0.6);
        }

        SmartDashboard.putNumber("angle Command", angleCommand/Math.PI*180);

        angleErr = MathUtil.angleModulus(this.angleCommand - this.getPose().getRotation().getRadians());
        DangleErr = angleErr - lastAngleError;
        SmartDashboard.putNumber("angle error", angleErr/Math.PI*180);
        if(directOmega){
            omega_out = omega;
        }else{
            omega_out = angleErr *4;
        }
        
        // omega_out = omega_out * 12;
        
        SmartDashboard.putNumber("omegaOut", omega_out);
        omegaL2 = omega_out * (DriveConstants.FrameConstants.kWheelBase / 2.0);
        omegaW2 = omega_out * (DriveConstants.FrameConstants.kTrackWidth / 2.0);

        // Compute the constants used later for calculating speeds and angles
        A = strafe - omegaL2;
        B = strafe + omegaL2;
        C = forward - omegaW2;
        D = forward + omegaW2;

        /*
         * Compute the drive motor speeds
         * Constant values re-arranged to invert direction of drive controls
         * to work with inverted wpilib paths.
         * Positive Y is now left. Positive X is forward. Positive rotation is
         * counter-clockwise.
         */
        speedFL = speed(B, C);
        speedBL = speed(A, C);
        speedFR = speed(B, D);
        speedBR = speed(A, D);


        deltaFL = speedFL - lastSpeedFL;
        deltaFR = speedFR - lastSpeedFR;
        deltaBL = speedBL - lastSpeedBL;
        deltaBR = speedBR - lastSpeedBR;

        accelFL = Math.abs(deltaFL) / dt;
        accelFR = Math.abs(deltaFR) / dt;
        accelBL = Math.abs(deltaBL) / dt;
        accelBR = Math.abs(deltaBR) / dt;


        forceFL = 60 * accelFL / 4; // Divide by 4 for four wheels
        forceFR = 60 * accelFR / 4;
        forceBL = 60 * accelBL / 4;
        forceBR = 60 * accelBR / 4;

        // Calculate current draw for each motor
        currentFL = (forceFL * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentFR = (forceFR * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentBL = (forceBL * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;
        currentBR = (forceBR * ModuleConstants.kWheelDiameterMeters/2) / ModuleConstants.kWheelTorqueConstantNmPerA;

        // Total current draw
        totalCurrent = currentFL + currentFR + currentBL + currentBR;
        if(totalCurrent > 170 && totalCurrent > 0){
            currentRatio =  170 / totalCurrent ; // scale down accelerations
            deltaFL *= currentRatio;
            deltaFR *= currentRatio;
            deltaBL *= currentRatio;
            deltaBR *= currentRatio;

            speedFL = lastSpeedFL + deltaFL;
            speedFR = lastSpeedFR + deltaFR;
            speedBL = lastSpeedBL + deltaBL;
            speedBR = lastSpeedBR + deltaBR;
        }
        /*
         * Compute the steer motor positions
         * Constant values re-arranged to invert direction of steer motor controls
         * to work with inverted wpilib paths.
         * Positive Y is now left. Positive X is forward. Positive rotation is
         * counter-clockwise.
         * NOTE: The letter sets in the speed and angle sections MUST MATCH
         */
        angleFL = angle(B, C);
        angleBL = angle(A, C);
        angleFR = angle(B, D);
        angleBR = angle(A, D);

        /*
         * Compute the maximum speed so that we can scale all the speeds to the range
         * [0.0, 1.0]
         */
        maxSpeed = Collections.max(Arrays.asList(Math.abs(speedFL), 
                                                        Math.abs(speedBL), 
                                                        Math.abs(speedFR), 
                                                        Math.abs(speedBR)));
        if(maxSpeed > DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond){
            double speedratio = maxSpeed / DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
            speedFL = speedFL / speedratio;
            speedFR = speedFR / speedratio;
            speedBL = speedBL / speedratio;
            speedBR = speedBR / speedratio;
        }

        if (deadStick) {
            // System.out.println("Dead Stick - Stopping Modules");
            frontLeft.setDriveSpeed(0);
            frontRight.setDriveSpeed(0);
            backLeft.setDriveSpeed(0);
            backRight.setDriveSpeed(0);

            frontLeft.setSteerSpeed(0);
            frontRight.setSteerSpeed(0);
            backLeft.setSteerSpeed(0);
            backRight.setSteerSpeed(0);

        } else {

            /*
             * Set each swerve module, scaling the drive speeds by the maximum speed
             */

            frontLeft.setSwerve(angleFL, speedFL , this._driveCorrect);
            frontRight.setSwerve(angleFR, speedFR , this._driveCorrect);
            backLeft.setSwerve(angleBL, speedBL , this._driveCorrect);
            backRight.setSwerve(angleBR, speedBR , this._driveCorrect);
        }
        lastSpeedFL =speedFL ;
        lastSpeedFR =speedFR ;
        lastSpeedBL =speedBL ;
        lastSpeedBR =speedBR ;
        // SmartDashboard.putNumber("speedfl", speedFL);
        // getSteerEncoderVal();
        lastInputTime = Timer.getFPGATimestamp();
    }

    private double speed(double val1, double val2) {
        return Math.hypot(val1, val2);
    }

    private double angle(double val1, double val2) {
        return Math.toDegrees(Math.atan2(val1, val2));
    }

    public double[] getDriveEncoders() {
        double[] values = new double[] {
                frontLeft.getDriveEncoder(),
                frontRight.getDriveEncoder(),
                backLeft.getDriveEncoder(),
                backRight.getDriveEncoder()
        };

        return values;
    }

    public double getDriveEncoderAvg() {
        double driveFL = Math.abs(frontLeft.getDriveEncoder());
        double driveFR = Math.abs(frontRight.getDriveEncoder());
        double driveBL = Math.abs(backLeft.getDriveEncoder());
        double driveBR = Math.abs(backRight.getDriveEncoder());
        return (driveFL + driveFR + driveBL + driveBR) / 4.0;
    }

    public void setDriveEncodersPosition(double position) {
        frontLeft.setDriveEncoder(position);
        frontRight.setDriveEncoder(position);
        backLeft.setDriveEncoder(position);
        backRight.setDriveEncoder(position);
    }

    public void getSteerEncoderVal() {
        SmartDashboard.putNumber("angleLF", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("angleRF", frontRight.getTurningPosition());
        SmartDashboard.putNumber("angleLB", backLeft.getTurningPosition());
        SmartDashboard.putNumber("angleRB", backRight.getTurningPosition());
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void addVisionMeasurement(Pose2d poseEst, double timestamp) {
        robotPoseEstimate.addVisionMeasurement(poseEst, timestamp);
    }

    public void addVisionMeasurement(Pose2d poseEst, double timestamp, Matrix<N3, N1> stdDevs) {
        robotPoseEstimate.addVisionMeasurement(poseEst, timestamp, stdDevs);    
    }

    

    public Rotation2d getSimRotation2d(){
        // SwerveModuleState[] states = new SwerveModuleState[4];
        // states[0] = frontLeft. getSimState();
        // states[1] = frontRight.getSimState();
        // states[2] = backLeft.  getSimState();
        // states[3] = backRight. getSimState();
        // var ChassisSpeeds = DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(states);
        // System.out.printf("%f %f\n",omega_out,ChassisSpeeds.omegaRadiansPerSecond);
        return new Rotation2d(angleCommand);
    }
    @Override()
    public void periodic() {
        
        if (!RobotBase.isSimulation()) {
            robotPoseEstimate.update(this._gyro.getRotation2d(), getPositions());
        }else{
            // Rotation2d simangle = robotPoseEstimate.getEstimatedPosition().getRotation().plus(getSimRotation2d().times(0.0002));
            Rotation2d simangle = new Rotation2d(angleCommand);
            var sim_positions = new SwerveModulePosition[] {
                frontLeft. getSimPosition(),
                frontRight.getSimPosition(),
                backLeft.  getSimPosition(),
                backRight. getSimPosition()
            };
            robotPoseEstimate.update(simangle,sim_positions);
        }
        // Pose2d tempPose = getPose();
        // double currentTime = Timer.getFPGATimestamp();
        // double dt = currentTime - robotVelEstimateLastTime;
        // robotVelEstimate =  (tempPose.minus(robotvelEsitmateLastPose).div(dt));
        // robotVelEstimateLastTime = currentTime;
        // robotvelEsitmateLastPose = tempPose;
        PPLibTelemetry.setCurrentPose(getPose());
        Pose2d RobotPose = robotPoseEstimate.getEstimatedPosition();
        SmartDashboard.putNumber("x_field", RobotPose.getX());
        SmartDashboard.putNumber("y_field", RobotPose.getY());
        SmartDashboard.putNumber("theta_field", RobotPose.getRotation().getDegrees());
        // if(Timer.getFPGATimestamp() - lastInputTime>0.02){
        //     frontLeft.setDriveSpeed(0);
        //     frontRight.setDriveSpeed(0);
        //     backLeft.setDriveSpeed(0);
        //     backRight.setDriveSpeed(0);

        //     frontLeft.setSteerSpeed(0);
        //     frontRight.setSteerSpeed(0);
        //     backLeft.setSteerSpeed(0);
        //     backRight.setSteerSpeed(0);
        // }
        
        // dont allow data input if it's a competition
        // boolean isCompetition = DriverStation.isFMSAttached() && DriverStation.isAutonomousEnabled();
        // if (isCompetition) {
        //     return;
        // }
        // query for input data
                                                                                                          // y,
                                                                                                                     // degrees
       

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
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Set Drive mode for balance Auto
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

    public void disableRamping() {
        frontLeft.driveMotorRamp(false);
        frontRight.driveMotorRamp(false);
        backLeft.driveMotorRamp(false);
        backRight.driveMotorRamp(false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        var speedFL = desiredStates[0].speedMetersPerSecond;
        var speedFR = desiredStates[1].speedMetersPerSecond;
        var speedBL = desiredStates[2].speedMetersPerSecond;
        var speedBR = desiredStates[3].speedMetersPerSecond;
        // double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR,
        //         Constants.DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond));

        frontLeft.setSwerve(desiredStates[0].angle.getDegrees(), speedFL , this._driveCorrect);
        frontRight.setSwerve(desiredStates[1].angle.getDegrees(), speedFR,  this._driveCorrect);
        backLeft.setSwerve(desiredStates[2].angle.getDegrees(), speedBL , this._driveCorrect);
        backRight.setSwerve(desiredStates[3].angle.getDegrees(), speedBR , this._driveCorrect);

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        return states;
    }

    private boolean isCoastMode = false;

    public boolean toggleMode() {
        return isCoastMode;
    }
    public void setup_shooter_solver_link(Supplier<Double> ShooterSolverYawSupplier,
    Supplier<Boolean> ShooterSolverSolutionFoundSupplier){
        this.ShooterSolverYawSupplier = ShooterSolverYawSupplier;
        this.ShooterSolverSolutionFoundSupplier = ShooterSolverSolutionFoundSupplier;
    }
    // TODO : Test SysID methods below Note: REALLY BROKEN
    /***********************************************************************************************
     * 
     * SYS ID TESTING METHODS BELOW
     * 
     * 
     * **********************************************************************************************
     */
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

     private final SysIdRoutine m_sysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
      ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    System.out.printf("%f %f\n", voltage.abs(Volts),frontLeft.getVoltage());
                    // frontLeft.setDriveVoltage(voltage.abs(Volts));
                    // frontRight.setDriveVoltage(voltage.abs(Volts));
                    // backLeft.setDriveVoltage(voltage.abs(Volts));
                    // backRight.setDriveVoltage(voltage.abs(Volts));
                    frontLeft.setSwerve(0,  4*voltage.abs(Volts)/RobotController.getBatteryVoltage() , this._driveCorrect);
                    frontRight.setSwerve(0, 4*voltage.abs(Volts)/RobotController.getBatteryVoltage() , this._driveCorrect);
                    backLeft.setSwerve(0,   4*voltage.abs(Volts)/RobotController.getBatteryVoltage() , this._driveCorrect);
                    backRight.setSwerve(0,  4*voltage.abs(Volts)/RobotController.getBatteryVoltage() , this._driveCorrect);
                },
                 log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-Front left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.getVoltage() , Volts))
                    .linearPosition(m_distance.mut_replace(frontLeft.getDriveEncoder(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeft.getDriveVelocity(), MetersPerSecond));
                log.motor("drive-Front Right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.getVoltage() , Volts))
                    .linearPosition(m_distance.mut_replace(frontRight.getDriveEncoder(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRight.getDriveVelocity(), MetersPerSecond));
                log.motor("drive-Back Left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backLeft.getVoltage() , Volts))
                    .linearPosition(m_distance.mut_replace(backLeft.getDriveEncoder(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backLeft.getDriveVelocity(), MetersPerSecond));
                log.motor("drive-Back Right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRight.getVoltage() , Volts))
                    .linearPosition(m_distance.mut_replace(backRight.getDriveEncoder(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backRight.getDriveVelocity(), MetersPerSecond));
               
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
            
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}