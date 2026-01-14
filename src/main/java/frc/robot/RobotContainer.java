// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Locomotion.Drive;
import frc.robot.subsystems.ultilities.Camera;
import frc.robot.subsystems.ultilities.Gyros.Gyro;
import frc.robot.subsystems.ultilities.Gyros.NavXGyro;
import frc.robot.subsystems.ultilities.Gyros.PigeonGyro;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.flowerTest;

import java.util.List;
import java.util.function.Supplier;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.geometry.Pose2d;
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
    // Field used for visualizing robot pose / path planner data on Shuffleboard
    
    private final Field2d field;

    // Subsystem singletons (exposed as statics for convenience in this project)
    public static Gyro _gyro; // the chosen gyro implementation
    public static Drive _drive; // drivetrain subsystem

    // Camera helpers - the list length/config is driven by Constants
    public static List<Camera> _cameras = new ArrayList<>();
    private boolean cameras_paused = false;

    // Command/utility used for PID tuning in this project (flowerTest)
    public flowerTest PID_TUNING;

    // Operator input device(s)
    // Replace with CommandPS4Controller or CommandXboxController if you change controllers
    private final CommandJoystick driverController;

    // Drive default command instance
    private static DriveCommand _driveCommand;
    // private final CommandXboxController operatorController = new
    // CommandXboxController(OperatorConstants.OpController);
// Robot pose supplier
    Supplier<Pose2d> robotPoseSupplier = () -> _drive.getPose();

    // Robot velocity supplier
    Supplier<Transform2d> robotVelocitySupplier = () -> _drive.getSpeedsWorld(true);

    private shooter _shooter_subsystem;

    
    // Setup Sendable chooser for picking autonomous program in SmartDashboard
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(int gyroType) {
       /****************************************************************
        * Initialize Gyro based on input parameter
        *****************************************************************/
        if (gyroType == Constants.NAVXGYROENUM) {
            _gyro = NavXGyro.getInstance();
        } else if (gyroType == Constants.PIGEONGYROENUM) {
            _gyro = PigeonGyro.getInstance();
        } else {
            _gyro = PigeonGyro.getInstance();
        }
        
        /****************************************************************
         * Initialize Drive subsystem
         *****************************************************************/
        _drive = Drive.getInstance(_gyro);
        
         /****************************************************************
         * Initialize Camera Subsystems
         *****************************************************************/
        _cameras.add(new Camera(Constants.CameraConstants.CAMERA1.getName(),
        Constants.CameraConstants.CAMERA1.getRobotToCamera(),
        _drive::addVisionMeasurement));
        _cameras.add(new Camera(Constants.CameraConstants.CAMERA2.getName(),
        Constants.CameraConstants.CAMERA2.getRobotToCamera(),
        _drive::addVisionMeasurement));
        
        Supplier <Pose2d> robPoseGetter = () ->_drive.getPose();
        _cameras.get(0).startGuessingOwnLoca(robPoseGetter);
        _cameras.get(1).startGuessingOwnLoca(robPoseGetter);

        
         /****************************************************************
         * Initialize Field Visualization
         *****************************************************************/
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        
        // Configure PathPlanner logging callbacks for pose and path visualization
        // Logs current robot pose to the field display
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(_drive.getPose());
        });

        // Logs target pose for path planning to the field display
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logs active path trajectory to the field display
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
        
         /****************************************************************
         * Initialize PID tuning utility for drive system
         *****************************************************************/
        PID_TUNING = new flowerTest(_drive);
        PID_TUNING.setRunning(true);

        _drive.addPIDTUNING(PID_TUNING);
        
        /****************************************************************
         * Initialize driver controller for manual input
         *****************************************************************/
        driverController = new CommandJoystick(OperatorConstants.DriveController);

        /****************************************************************
         * Register named commands for PathPlanner auto routines
         *****************************************************************/
        configureNamedCommands();

        /****************************************************************
         * Load and configure autonomous options from chooser
         *****************************************************************/
        autonomousOptions();

        /****************************************************************
         * Configure input device bindings
         *****************************************************************/
        configureBindings();
        
        /****************************************************************
         * Configure default drive command and bindings for driver control
         *****************************************************************/
        _driveCommand = new DriveCommand(_drive, driverController);
        CommandScheduler.getInstance()
                .setDefaultCommand(_drive, _driveCommand);
        _drive.angleCommandNotSet = true;  
        
        /****************************************************************
         * Initialize shooter subsystem, and link to drive subsystem
         *****************************************************************/
        _shooter_subsystem = shooter.getInstance(robotPoseSupplier, robotVelocitySupplier);
        Supplier<Double> ShooterSolverYawSupplier = () -> _shooter_subsystem.getRobotRequiredYawRad();
        Supplier<Boolean> ShooterSolverSolutionFoundSupplier = () -> _shooter_subsystem.isShootingAllowed();
        _drive.setup_shooter_solver_link(ShooterSolverYawSupplier, ShooterSolverSolutionFoundSupplier);
        
    }

    /****************************************************************
     * Configure input device bindings
    *****************************************************************/
    private void configureBindings() {
        // Zero Gyro on Drive B press
        this.driverController.button(2).onTrue(new InstantCommand(() -> this._shooter_subsystem.AllowShooting(true)));
        this.driverController.button(2).onFalse(new InstantCommand(() -> this._shooter_subsystem.AllowShooting(false)));
        this.driverController.button(3).onFalse(new InstantCommand(() -> this._shooter_subsystem.change_target(Constants.HopperPose)));
        // this.driverController.b().onTrue(new InstantCommand(() -> _drive.resetPoseRotation())); // changed this to reset
        //                                                                                         // pose rotation, so
        //                                                                                         // that field centric
        //                                                                                         // can tie to pose info
        
        // this.driverController.start().onTrue(new InstantCommand(() -> toggleCameraPoseUpdate())); // added this to give  
        //                                                                                           // driver a means of
        //                                                                                           // disabling camera
        //                                                                                           // updates IE bad
        //                                                                                           // camera
        
        


    }
    /****************************************************************
     * Configure SysId input device bindings
    *****************************************************************/
    @SuppressWarnings("unused")
    private void configureSysIdBindings(){
        boolean isCompetition = DriverStation.isFMSAttached() && DriverStation.isAutonomousEnabled();
        if (!isCompetition) {
            // this.driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        }
    }
    /****************************************************************
     * Configure Named Commands
    *****************************************************************/
    public void configureNamedCommands() {
        /****************************************************************
         * Shooter Commands
        *****************************************************************/
        NamedCommands.registerCommand("ShootAtTarget", new InstantCommand(() -> this._shooter_subsystem.AllowShooting(true)));
        NamedCommands.registerCommand("StopShoot",     new InstantCommand(() -> this._shooter_subsystem.AllowShooting(false)));
        
        NamedCommands.registerCommand("ChangeTargetHopper", new InstantCommand(() -> this._shooter_subsystem.change_target(Constants.HopperPose)));
        
        NamedCommands.registerCommand("ChangeTargetNearFieldLeft", new InstantCommand(() ->  this._shooter_subsystem.change_target(Constants.nearfieldLeftPose)));
        NamedCommands.registerCommand("ChangeTargetNearFieldRight", new InstantCommand(() -> this._shooter_subsystem.change_target(Constants.nearfieldRightPose)));


    }

    /****************************************************************
     * getAutonomousCommand from chooser
    *****************************************************************/
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        
        return autoChooser.getSelected();
    }

    /****************************************************************
     * Use this to set Autonomous options for selection in Smart Dashboard
    *****************************************************************/
    private void autonomousOptions() {
        // Example adding Autonomous option to chooser
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        autoChooser.addOption("PID TUNING", PID_TUNING);
        SmartDashboard.putData("Auto Mode", autoChooser);

    }
    /****************************************************************
     *  toggle camera pose updater
    *****************************************************************/
    @SuppressWarnings("unused")
    private void toggleCameraPoseUpdate() {
        this.cameras_paused = !this.cameras_paused;
        for (var camera : _cameras) {
            camera.pause_periodic(this.cameras_paused);
        }
    }


}
