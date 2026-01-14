// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Locomotion.Drive;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Processes gamepad input for teleoperated swerve drive control.
// Handles joystick scaling, field/robot-centric mode selection, dead zone filtering, and speed ramping.
// Forwards processed velocity commands to the Drive subsystem for execution.
public class DriveCommand extends Command {

    private Drive _drive;
    private CommandJoystick _driveController; // Primary input device for driving (left stick: forward/strafe, right stick: rotation)
    private JoystickButton _leftBumper;  // Turbo mode (100% speed)
    private JoystickButton _rightBumper; // Precision mode (25% speed)

    // Joystick control constants
    public static final double OMEGA_SCALE = 1.0 / 20.0; // Scaling factor for rotation speed to limit yaw rate
    public static final double DEADZONE_LSTICK = 0.1; // Left joystick deadzone; prevents drift from stick centering errors
    private static final double DEADZONE_RSTICK = 0.1; // Right joystick deadzone for rotation

    // Joystick curve exponents: Higher values create "soft" response near center, sharp response at extremes.
    // This allows precise control at low speeds while maintaining quick response at full deflection.
    private double leftPow = 1;  // Exponent for forward/strafe (currently linear)
    private double rightPow = 1; // Exponent for rotation (currently linear)

    private boolean deadStick; // Flag: true when all joysticks are within deadzone (prevents motor updates)
    private Translation2d FieldVelocity; // Desired velocity vector in field coordinates (x, y)
    private Translation2d RobotVelocity; // Desired velocity vector in robot coordinates (x, y)

    // Constructor: Initializes the drive command with the swerve drive subsystem and input controller.
    // Sets up bumper buttons for speed mode selection: left bumper (turbo) and right bumper (precision).
    public DriveCommand(Drive drive, CommandJoystick driveController) {
        this._drive = drive;
        this._driveController = driveController;
        this._leftBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kLeftBumper.value);
        this._rightBumper = new JoystickButton(this._driveController.getHID(),
                XboxController.Button.kRightBumper.value);
       
        this.deadStick = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Initializes PID controllers for autonomous-style drive adjustments.
    // These controllers are used when the command needs precise velocity control with feedback.
    // Continuous input for rotation allows smooth wrapping across -180° to 180° boundary.
    @Override
    public void initialize() {
        // Initialize PID controllers here if needed
    }

    // Processes joystick input and forwards velocity commands to the drive subsystem every scheduler cycle.
    @Override
    public void execute() {
        // Get field-centric mode toggle: Button 1 enables field-centric; when false, use robot-centric
        boolean stickFieldCentric = _driveController.button(1).getAsBoolean();

        // Speed mode selection via bumpers:
        // - Left bumper: 100% speed (turbo mode)
        // - Right bumper: 25% speed (precision mode)
        // - Neither: 60% speed (default)
        double speedMultiplier;
        if (_leftBumper.getAsBoolean()) {
            speedMultiplier = 1.0;
        } else if (_rightBumper.getAsBoolean()) {
            speedMultiplier = 0.25;
        } else {
            speedMultiplier = 0.60;
        }

        // Extract raw joystick inputs and apply speed scaling.
        // Inverted (negated) to align drive directions with PathPlanner's coordinate system.
        // Deadzone filtering prevents motor noise from stick centering errors.
        double stickForward = -MathUtil.applyDeadband(this._driveController.getY() * speedMultiplier, DEADZONE_LSTICK);
        double stickStrafe = -MathUtil.applyDeadband(this._driveController.getX() * speedMultiplier, DEADZONE_LSTICK);
        double stickOmega = -MathUtil.applyDeadband(this._driveController.getTwist() * speedMultiplier, DEADZONE_RSTICK);

        // Apply joystick curve exponents to enable fine control near center and sharp response at extremes.
        // Math.pow(|value|, exponent) raises magnitude to a power, then Math.signum restores the sign.
        // Exponent of 1.0 = linear response, > 1.0 = curved (soft start, sharp end).
        double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe)*DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
        double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward)*DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
        // Rotation is scaled by 720°/s max rate to limit turn speed and prevent yaw instability
        double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(stickOmega) * Math.toRadians(720);

        // Field-centric mode: Rotate joystick inputs from driver's perspective to field coordinates.
        // This makes the robot move in absolute directions regardless of its heading.
        // For example: "forward" always means toward the wall, not toward robot's nose.
        // Robot-centric mode: Forward moves in the direction the robot is facing.
        if (!stickFieldCentric) { // Field-centric enabled
            // Create velocity vector in field coordinates using raw joystick input
            FieldVelocity = new Translation2d(forward, strafe);
            
            // Adjust for alliance color: Red alliance rotates field coordinates 180° for consistency
            var alliance = DriverStation.getAlliance();
            if (alliance.get() == DriverStation.Alliance.Red) {
                FieldVelocity = FieldVelocity.rotateBy(Rotation2d.fromDegrees(180));
            }
            
            // Convert field velocity to robot velocity by rotating opposite to robot's current heading.
            // This transforms from "field perspective" back to "robot perspective" for the drive subsystem.
            RobotVelocity = FieldVelocity.rotateBy(_drive.getPose().getRotation().unaryMinus());
            
            forward = RobotVelocity.getX();
            strafe = RobotVelocity.getY();
        }

        // Check if all joysticks are in deadzone (dead stick = no movement commanded).
        // Flag prevents unnecessary motor updates and reduces electrical noise when stationary.
        if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
            this.deadStick = true;
        } else {
            this.deadStick = false;
        }

        // Send the processed velocity command to the drive subsystem for execution.
        // Pass deadStick flag to allow drive to optimize power when robot is stationary.
        this._drive.processInput(forward, strafe, omega, this.deadStick, false, false);
    }

    // Called once the command ends or is interrupted. No cleanup needed for drive command.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end (never, since this is the default drive command).
    @Override
    public boolean isFinished() {
        return false;
    }

    // Public accessor for deadstick status; used by other commands to detect operator inactivity.
    public boolean getIsDeadStick() {
        return this.deadStick;
    }

}