// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Locomotion.Drive;
import frc.robot.subsystems.ultilities.Camera;
import frc.robot.subsystems.ultilities.FollowPathCommand103;
import frc.robot.subsystems.ultilities.Gyros.Gyro;
import frc.robot.subsystems.ultilities.Gyros.NavXGyro;
import frc.robot.subsystems.ultilities.Gyros.PigeonGyro;
import frc.robot.subsystems.ultilities.MovingPlatformShooterSolver.Shooter_Solver;
import frc.robot.simulation.ShooterSolverSimulation;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.flowerTest;
import frc.robot.commands.autos.DriveForwardSlow;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
import java.util.function.Supplier;
import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
// import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // path planner logging
    private final Field2d field;
    // TODO Remove SIMULATION LINE 
    private final ShooterSolverSimulation shooterSolverSim = new ShooterSolverSimulation();
    // declaration of subclasses
    public static Gyro _gyro;
    public static Drive _drive;
    public static List<Camera> _cameras = new ArrayList<>();
    private boolean cameras_paused = false;
    public flowerTest PID_TUNING;
    private double[] HopperPoseArray = new double[] {Units.inchesToMeters(180),
                                                                    Units.inchesToMeters(157.32), 
                                                                    Units.inchesToMeters(50)};
    private double[] nearfieldTargetPoseArray = new double[] {Units.inchesToMeters(110),
                                                                    Units.inchesToMeters(165.32), 
                                                                    Units.inchesToMeters(70)};
    private double[] nearfieldLeftPoseArray = new double[] {Units.inchesToMeters(110),
                                                                    Units.inchesToMeters(210.32), 
                                                                    Units.inchesToMeters(70)};
    private double[] nearfieldRightPoseArray = new double[] {Units.inchesToMeters(110),
                                                                    Units.inchesToMeters(90.32), 
                                                                    Units.inchesToMeters(70)};
    private double[] targetPoseArray = SmartDashboard.getNumberArray("Target Robot Pose", new double[] {HopperPoseArray[0],
                                                                     HopperPoseArray[1], 
                                                                     HopperPoseArray[2]});
                                                                    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController;
    private static DriveCommand _driveCommand;
    // private final CommandXboxController operatorController = new
    // CommandXboxController(OperatorConstants.OpController);
    private Shooter_Solver _Shooter_Solver;
    private Shooter_Solver _Shooter_Solver2;
    // Setup Sendable chooser for picking autonomous program in SmartDashboard
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(int gyroType) {
        // path planner logging
        // shooterSolverSim.main(new String[]{}); // TODO Remove SIMULATION LINE
        // input integer to define the gyro callbacks that will be used for the project
        // Gryo is an interface for the 2 classes NavXGyro and PigeonGyro, which
        // implement the required functions
        if (gyroType == Constants.NAVXGYROENUM) {
            _gyro = NavXGyro.getInstance();
        } else if (gyroType == Constants.PIGEONGYROENUM) {
            _gyro = PigeonGyro.getInstance();
        } else {
            _gyro = PigeonGyro.getInstance();
        }
        // the gyro is them passed into drive, which needs it for the constructor
        _drive = Drive.getInstance(_gyro);
        // cameras are initialized based on how many parameters are in the constants
        // file
        // TODO : Clean up camera configuration to be more dynamic if more cameras
        _cameras.add(new Camera(Constants.CameraConstants.CAMERA1.getName(),
        Constants.CameraConstants.CAMERA1.getRobotToCamera(),
        _drive::addVisionMeasurement));
        _cameras.add(new Camera(Constants.CameraConstants.CAMERA2.getName(),
        Constants.CameraConstants.CAMERA2.getRobotToCamera(),
        _drive::addVisionMeasurement));
        
        Supplier <Pose2d> robPoseGetter = () ->_drive.getPose();
        _cameras.get(0).startGuessingOwnLoca(robPoseGetter);
        _cameras.get(1).startGuessingOwnLoca(robPoseGetter);

        System.out.printf("%s\n\n",_cameras.get(0).name);
        System.out.printf("%s\n\n",_cameras.get(1).name);
       
        field = new Field2d();
     
        SmartDashboard.putData("Field", field);
        
     
        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(_drive.getPose());
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
        
        PID_TUNING = new flowerTest(_drive);
        PID_TUNING.setRunning(true);
        // SmartDashboard.putBoolean("start_pid_tuning", false);
        _drive.addPIDTUNING(PID_TUNING);
        driverController = new CommandXboxController(OperatorConstants.DriveController);
        // Configured Named commands for pathplanner
        configureNamedCommands();

        // Configure Autonomous Options
        autonomousOptions();
        // CommandScheduler.getInstance().schedule(FollowPathCommand103.warmupCommand());
        // PathfindingCommand.warmupCommand().schedule();
        // Configure the trigger bindings
        configureBindings();
        
        // Default Comands always runningd
        _driveCommand = new DriveCommand(_drive, driverController);
        CommandScheduler.getInstance()
                .setDefaultCommand(_drive, _driveCommand);
        _drive.angleCommandNotSet = true;   
        setupShooterSolver();
        Supplier<Double> ShooterSolverYawSupplier = () -> _Shooter_Solver.getRobotRequiredYawRad();
        Supplier<Boolean> ShooterSolverSolutionFoundSupplier = () -> _Shooter_Solver.getBestSolutionStatus().equals("Solution Found");
        _drive.setup_shooter_solver_link(ShooterSolverYawSupplier, ShooterSolverSolutionFoundSupplier);
        
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Zero Gyro on Drive B press
        
        this.driverController.b().onTrue(new InstantCommand(() -> _drive.resetPoseRotation())); // changed this to reset
                                                                                                // pose rotation, so
                                                                                                // that field centric
                                                                                                // can tie to pose info
        // TODO: TEST this toggle works
        this.driverController.start().onTrue(new InstantCommand(() -> toggleCameraPoseUpdate())); // added this to give  
                                                                                                  // driver a means of
                                                                                                  // disabling camera
                                                                                                  // updates IE bad
                                                                                                  // camera
        
        boolean isCompetition = DriverStation.isFMSAttached() && DriverStation.isAutonomousEnabled();


    }
    @SuppressWarnings("unused")
    private void configureSysIdBindings(){
        boolean isCompetition = DriverStation.isFMSAttached() && DriverStation.isAutonomousEnabled();
        if (!isCompetition) {
            this.driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
            this.driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

            /*
            * Joystick Y = quasistatic forward
            * Joystick A = quasistatic reverse
            * Joystick B = dynamic forward
            * Joystick X = dyanmic reverse
            */
            this.driverController.y().whileTrue(_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            this.driverController.a().whileTrue(_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            this.driverController.b().whileTrue(_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            this.driverController.x().whileTrue(_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
    }

    public void configureNamedCommands() {

        NamedCommands.registerCommand("ShootAtTarget", new InstantCommand(() -> this.AllowShooting(true)));
        NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> this.AllowShooting(false)));
        
        NamedCommands.registerCommand("ChangeTargetHopper", new InstantCommand(() -> this.change_target(HopperPoseArray[0], HopperPoseArray[1], HopperPoseArray[2])));
        NamedCommands.registerCommand("ChangeTargetNearField", new InstantCommand(() -> this.change_target(nearfieldTargetPoseArray[0], nearfieldTargetPoseArray[1], nearfieldTargetPoseArray[2])));
        NamedCommands.registerCommand("ChangeTargetNearFieldLeft", new InstantCommand(() -> this.change_target(nearfieldLeftPoseArray[0], nearfieldLeftPoseArray[1], nearfieldLeftPoseArray[2])));
        NamedCommands.registerCommand("ChangeTargetNearFieldRight", new InstantCommand(() -> this.change_target(nearfieldRightPoseArray[0], nearfieldRightPoseArray[1], nearfieldRightPoseArray[2])));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        
        return autoChooser.getSelected();
    }

    /**
     * Use this to set Autonomous options for selection in Smart Dashboard
     */
    private void autonomousOptions() {
        // Example adding Autonomous option to chooser
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        autoChooser.addOption("PID TUNING", PID_TUNING);
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    private void toggleCameraPoseUpdate() {
        this.cameras_paused = !this.cameras_paused;
        for (var camera : _cameras) {
            camera.pause_periodic(this.cameras_paused);
        }
    }




    /*
     * 
     *  Shooter solver section
     * 
     */
    public void change_target(double fieldx, double fieldy, double fieldz){
            targetPoseArray =  new double[] {fieldx,
            fieldy, 
            fieldz};
    }
    private double target_offset_from_center = Units.inchesToMeters(6);
    private Pose3d targetPoseSup(){
        
        double targetX = targetPoseArray[0];
        double targetY = targetPoseArray[1]+target_offset_from_center;
        double targetZ = targetPoseArray[2];
        return new Pose3d(new Translation3d(targetX, targetY, targetZ), new Rotation3d(0,0,0));
    }
    private Pose3d targetPose2Sup(){
       
        double targetX = targetPoseArray[0];
        double targetY = targetPoseArray[1] - target_offset_from_center;
        double targetZ = targetPoseArray[2];
        return new Pose3d(new Translation3d(targetX, targetY, targetZ), new Rotation3d(0,0,0));
    }
    private void AllowShooting(boolean allow){
        _Shooter_Solver.AllowShooting(allow);
        _Shooter_Solver2.AllowShooting(allow);
    }
    @SuppressWarnings("unused")
    private void setupShooterSolver(){
         
        Supplier<Pose3d> targetPoseSupplier = () -> targetPoseSup();
        Supplier<Pose3d> targetPose2Supplier = () -> targetPose2Sup();

        // Robot pose supplier
        Supplier<Pose2d> robotPoseSupplier = () -> _drive.getPose();

        // Robot velocity supplier
        Supplier<Transform2d> robotVelocitySupplier = () -> _drive.getVelocityWorld();

        // Shooter-to-robot transform supplier
        Translation3d shooterRelativePosition = new Translation3d(0.3, 0.0, 0.5);
        Translation3d shooter2RelativePosition = new Translation3d(-0.3, 0.0, 0.5);
        double shooterYawtoRobot = 0.0;
        double shooter2YawtoRobot = 0.0;
        Supplier<Transform3d> shooterTransformSupplier = () -> new Transform3d(shooterRelativePosition, new Rotation3d(0,0,Math.toRadians(shooterYawtoRobot)));
        Supplier<Transform3d> shooter2TransformSupplier = () -> new Transform3d(shooter2RelativePosition, new Rotation3d(0,0,Math.toRadians(shooter2YawtoRobot)));

        double shooterMuzzleV0 = 8.0; // Muzzle velocity (m/s)
        double shooter2MuzzleV0 = 9.0; // Muzzle velocity (m/s)

        SmartDashboard.putNumber("Shooter Solver Muzzle V0", shooterMuzzleV0);
        Supplier<Double> shooterMuzzleV0Supplier = () -> SmartDashboard.getNumber("Shooter Solver Muzzle V0", shooterMuzzleV0);
        SmartDashboard.putNumber("Shooter Solver2 Muzzle V0", shooter2MuzzleV0);
        Supplier<Double> shooter2MuzzleV0Supplier = () -> SmartDashboard.getNumber("Shooter Solver2 Muzzle V0", shooter2MuzzleV0);
        
        // Create the Shooter_Solver instance
        _Shooter_Solver = Shooter_Solver.getInstance(1,
        robotPoseSupplier,
        robotVelocitySupplier,
        shooterTransformSupplier,
        targetPoseSupplier,
        shooterMuzzleV0Supplier
        );
        _Shooter_Solver.getBestSolutionStatus(); // warmup and get rid of java warnings
        _Shooter_Solver2 = Shooter_Solver.getInstance(2,
        robotPoseSupplier,
        robotVelocitySupplier,
            shooter2TransformSupplier,
            targetPose2Supplier,
            shooter2MuzzleV0Supplier
            );
            _Shooter_Solver2.getBestSolutionStatus(); // warmup and get rid of java warnings
        SmartDashboard.putNumber("Shooter Solver Muzzle V0", shooterMuzzleV0);
        SmartDashboard.putNumber("Shooter Solver2 Muzzle V0", shooter2MuzzleV0);
    }
    
    
}
