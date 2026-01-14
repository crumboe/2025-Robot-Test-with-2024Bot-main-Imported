package frc.robot;


import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.LinearVelocity;
// import frc.robot.Constants.DriveConstants.FrameConstants;
//import frc.robot.utility.AprilTag;

// import static edu.wpi.first.units.Units.*;


public class Constants {
    public static final int NAVXGYROENUM = 0;
    public static final int PIGEONGYROENUM =1;
    public static final double PP103DistErrorLim = 2;
    public static class OperatorConstants {
        public static final int DriveController = 0;
        public static final int OpController = 1;
    }


    public static class DriveConstants {
        public static final int PigeonCANId = 5;

        public static final int FrontLeftSteer = 22;//20;
        public static final int FrontLeftDrive = 12;//10;
        public static final int FrontLeftEncoderOffset = 0;

        public static final int FrontRightSteer = 21;//23;
        public static final int FrontRightDrive = 11;//13;
        public static final int FrontRightEncoderOffset = 0;

        public static final int BackLeftSteer = 23;//21;
        public static final int BackLeftDrive = 13;//11;
        public static final int BackLeftEncoderOffset = 0;

        public static final int BackRightSteer = 20;//22;
        public static final int BackRightDrive = 10;//12;
        public static final int BackRightEncoderOffset = 0;


        public static final class ModuleConstants {
            
            public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 4
            public static final double kDriveMotorGearRatio = 1 / 6.429;

            // Invert the turning encoder if necessary. If pivots snap back and forth when setting up
            // inversion may be needed as the encoder can be reading in the opposite direction 
            // as the pivots are going. 
            public static final boolean kTurningEncoderInverted = false;

            // Calculations required for driving motor conversion factors and feed forward
            public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
            public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
            public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kDriveMotorGearRatio)
             * kWheelCircumferenceMeters;

            public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            * kDriveMotorGearRatio; // meters

            public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            * kDriveMotorGearRatio) / 60.0; // meters per second

            public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
            public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
        
            public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
            public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
        
            
            public static final double kDrivingP = .1;
            public static final double kDrivingI = 0;
            public static final double kDrivingD = 0;
            public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
            public static final double kDrivingMinOutput = -1;
            public static final double kDrivingMaxOutput = 1;
        
            public static final double kTurningP = 0.8;
            public static final double kTurningI = 0;
            public static final double kTurningD = 0.005;
            public static final double kTurningFF = 0;
            public static final double kTurningMinOutput = -1;
            public static final double kTurningMaxOutput = 1;
              // NEO motor datasheet values (REV NEO)
              public static final double kNeoStallTorqueNm = 2.6;     // N·m (stall torque)
              public static final double kNeoStallCurrentA = 105.0;  // A (stall current)
          
              // Motor torque constant at motor shaft (N·m per amp)
              public static final double kMotorTorqueConstantNmPerA = kNeoStallTorqueNm / kNeoStallCurrentA;
          
              // Gear reduction: compute motor-to-wheel reduction factor.
              // ModuleConstants.kDriveMotorGearRatio in this project is defined as (wheel revs) / (motor revs) = 1/6.429.
              // So motor revs per wheel rev = 1 / kDriveMotorGearRatio (≈ 6.429).
              public static final double kMotorToWheelReduction = 1.0 / kDriveMotorGearRatio;
          
              // Torque constant reflected to the wheel (N·m per amp at the wheel)
              public static final double kWheelTorqueConstantNmPerA = kMotorTorqueConstantNmPerA * kMotorToWheelReduction;
          
            

        }

        public static class FrameConstants {
            // Distance between right and left wheels
            public static final double bumper_length = Units.inchesToMeters(30);  // Comp Bot is 22;
            public static final double backtoWheelsCenter = Units.inchesToMeters(11.625);  // Comp Bot is 22;
            public static final double bumpercentertoWheelCenter = Units.inchesToMeters(5.625);  // Comp Bot is 22;
            public static final double WHEEL_BASE_WIDTH = 21.375;  // Comp Bot is 22;
            public static final double kTrackWidth = Units.inchesToMeters(WHEEL_BASE_WIDTH);

            // Distance between front and back wheels
            public static final double WHEEL_BASE_LENGTH = 17.3125; //Comp bot is 22;
            public static final double kWheelBase = Units.inchesToMeters(WHEEL_BASE_LENGTH);

            public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d frModuleOffset = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
            public static final Translation2d blModuleOffset = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d brModuleOffset = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                flModuleOffset,
                frModuleOffset,
                blModuleOffset,
                brModuleOffset
            );
            public static final double AngleP = .05;
            public static final double AngleD = .01;
            public static double kPhysicalMaxAcceleration                          = 3;
            public static double kphysicalMaxAngularAccelerationRadiansPerSecond   = 1* Math.PI;
            public static double kPhysicalMaxSpeedMetersPerSecond                  = 4.69; //6;
            public static double kPhysicalMaxAngularSpeedRadiansPerSecond          = 1.5 * Math.PI;
            public void setSpeedAndAccelConstants(double kMaxAcc,double kMaxAngAcc,double kMaxSpeed,double kMaxAngSpeed){
                kPhysicalMaxAcceleration                       =kMaxAcc;
                kphysicalMaxAngularAccelerationRadiansPerSecond=kMaxAcc;
                kPhysicalMaxSpeedMetersPerSecond               =kMaxSpeed;
                kPhysicalMaxAngularSpeedRadiansPerSecond       =kMaxAngSpeed;
            }
        }

        // public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
        public static PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
            new PIDConstants(10, 0.01, 0.1), // Translation constants P=5.0, I=0, D=0 kP=2.3
            new PIDConstants(7.5, 0, 0)); // Rotation constants P=0.68 P=5.0, I=0, D=0  kP=2.8 .65
    }
    


    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    
    // At the risk of complicating this camera related part of the constants, 
    // I have made it so that multiple camera parameters can be added here,
    //  and there should not be any other changes elsewhere in the code. CMR
    public static final double CAMERA_RELOCALIZE_GUESS_OWN_LOC_INTERVAL = 1.0; // seconds between relocalize when guessing own location
    public static final double ACCEPTABLE_CAMERA_POSE_UPDATE_DISTANCE = 20.0; // meters
    public enum CameraConstants{
        // initialize values for first camera, copy for second camera and so on
        CAMERA1("Arducam_OV9281_USB_Camera", 
            new Transform3d( new Translation3d( 
                -0.19,                                    // x rbt to cam in meters
                0,                                      // y rbt to cam in meters
                .36)                                    // z rbt to cam in meters
            , new Rotation3d(
                Units.degreesToRadians(0),        // roll
                Units.degreesToRadians(0),        // pitch
                Units.degreesToRadians(0)         // yaw
            ))),
        CAMERA2("OV9281BlueRight", 
            new Transform3d( new Translation3d( 
                .1,                                    // x rbt to cam in meters
                0,                                      // y rbt to cam in meters
                .36)                                    // z rbt to cam in meters
            , new Rotation3d(
                0.001,        // roll
                -.062,        // pitch
                 -3.01058        // yaw
            )));
            
            ;
        // cameraConstants underlying constructor and getters
        private String Name;
        private Transform3d RobotToCamera;
        // individual item constructor
        CameraConstants(String Name, Transform3d RobotToCamera){
            this.Name = Name;
            this.RobotToCamera = RobotToCamera;
        }
        // name getter
        public String getName(){
            return this.Name;
        }
        // R2C transform getter
        public Transform3d getRobotToCamera(){
            return RobotToCamera;
        }
    }
    
}