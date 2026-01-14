// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Gyros.Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Locomotion.Drive;

public class DriveCommand extends Command {

    private Drive _drive;
    // private CommandJoystick leftStick;
    // private CommandJoystick rightStick;
    private CommandXboxController _driveController;
    // private XboxController _driveController_HID = _driveController.getHID();
    private JoystickButton _leftBumper;
    private JoystickButton _rightBumper;
    // private Gyro _gyro;

    public static final double OMEGA_SCALE = 1.0 / 20.0;// 30.0;// 45
    public static final double DEADZONE_LSTICK = 0.1;
    private static final double DEADZONE_RSTICK = 0.1;
    // private double originHeading = 0.0;
    private double leftPow = 1;
    private double rightPow = 1;

    private PIDController _driveRotationPID, _driveDistancePID, _driveStrafePID;
    private double _driveRotationP = 0.0004, _driveRotationD = 0.00, _driveRotationI = 0.00;// p=0.0002
    private double _driveDistanceP = 0.015, _driveDistanceD = 0.008, _driveDistanceI = 0.00;// p=0.002
    private double _driveStrafeP = 0.015, _driveStrafeD = 0.00, _driveStrafeI = 0.00;// p=0.015 d=0.008

    private boolean deadStick;
    private Translation2d FieldVelocity;
    private Translation2d RobotVelocity;

    /*
     * Creates a new DriveCommand using a CommandXboxController for driving
     */
    public DriveCommand(Drive drive, CommandXboxController driveController) {
        this._drive = drive;
        this._driveController = driveController;
        this._leftBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kLeftBumper.value);
        this._rightBumper = new JoystickButton(this._driveController.getHID(),
                XboxController.Button.kRightBumper.value);
        // this._gyro = gyro;
        this.deadStick = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        /*
         * Get the starting position of the gyro.
         * This will be used as the initial angle of the robot for field centric
         * control.
         */

        // originHeading = _gyro.getZeroAngle();

        // _drive.setDrivesMode(IdleMode.kCoast);
        _driveRotationPID = new PIDController(_driveRotationP, _driveRotationI, _driveRotationD);
        _driveRotationPID.setTolerance(0.8);

        _driveRotationPID.enableContinuousInput(-180, 180);

        _driveDistancePID = new PIDController(_driveDistanceP, _driveDistanceI, _driveDistanceD);
        _driveDistancePID.setTolerance(2);

        _driveStrafePID = new PIDController(_driveStrafeP, _driveStrafeI, _driveStrafeD);
        _driveStrafePID.setTolerance(2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /*
         * Get the starting position of the gyro.
         * This will be used as the initial angle of the robot for field centric
         * control.
         */
        // originHeading = _gyro.getZeroAngle();

        // final double originOffset = 360 - originHeading;

        boolean stickFieldCentric;

        stickFieldCentric = _driveController.leftTrigger().getAsBoolean();

        /*
         * The sticks are being inverted in the following lines to work with the
         * revised drive code. The revised drive code sets aligns the drive axis
         * and drive directions to match the odd directions of wpilib. This is done
         * to allow the use of path planning software in autonoumous mode.
         */

        double stickForward;
        double stickStrafe;
        double stickOmega;
        this.deadStick = false;
        double speedMultiplier;
        if (_leftBumper.getAsBoolean()) {
            speedMultiplier = 1;
        } else if (_rightBumper.getAsBoolean()) {
            speedMultiplier = 0.25;
        } else {
            speedMultiplier = 0.60;
        }

        stickForward = -MathUtil.applyDeadband(this._driveController.getLeftY() * speedMultiplier, DEADZONE_LSTICK);// -
        stickStrafe = -MathUtil.applyDeadband(this._driveController.getLeftX() * speedMultiplier, DEADZONE_LSTICK);// -
        stickOmega = -MathUtil.applyDeadband(this._driveController.getRightX() * speedMultiplier, DEADZONE_RSTICK);

        /*
         * The following lines allow the programmer to increase or decrease the initial
         * joystick action. The input of the joystick is taken to a power. If the
         * exponent is
         * one then the joystick action is linear. If the power is two then the initial
         * action will be a "soft" ramp, while the ending action will sharply increase.
         */

        double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe);
        double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
        double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(stickOmega) * Math.toRadians(720);

        /*
         * When the Left Joystick trigger is not pressed, The robot is in Field Centric
         * Mode.
         * The calculations correct the forward and strafe values for field centric
         * attitude.
         * Rotate the velocity vector from the joystick by the difference between our
         * current orientation and the current origin heading.
         */
        if (!stickFieldCentric) { // New field centric command, incorportates pose heading informtion for more
                                  // "robust" sense of which direction should be north
            // Changed the zeroing button on the controller to 0 out the pose angle.
            var alliance = DriverStation.getAlliance();
            FieldVelocity = new Translation2d(forward, strafe).times(DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond); // create the target vector in space
                                                                              // X_field,
                                                                              // Y_field
            if (alliance.get() == DriverStation.Alliance.Red) {
                FieldVelocity = FieldVelocity.rotateBy(Rotation2d.fromDegrees(180)); // if on the other side, rotate
                                                                                     // field
                                                                                     // cordinates to match view.
            }
            RobotVelocity = FieldVelocity.rotateBy(_drive.getPose().getRotation().unaryMinus()); // Rotate our target vector around the robot's center, opposite to the robots heading
            // New vector should be in Space X_robot, Y_Robot

            forward = RobotVelocity.getX();
            strafe = RobotVelocity.getY();
        }

        /*
         * If all of the joysticks are in the deadzone, don't update the motors
         */
        if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
            this.deadStick = true;
        }


        /*
         * Opperate the path following command if there are dead sticks and a path is
         * supposed to be followed,
         * otherwise send the regular drive processinput command
         * 
         */
        if (RobotBase.isSimulation()){
            this.deadStick = true;
        }
        this._drive.processInput(forward, strafe, omega, this.deadStick,false,false); // No, drive as normal
       
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }


    public boolean getIsDeadStick() {
        return this.deadStick;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
