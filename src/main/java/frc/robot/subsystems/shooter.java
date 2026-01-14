package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ultilities.MovingPlatformShooterSolver.Shooter_Solver;
import frc.robot.Constants;

/****************************************************************
 * Shooter Subsystem Definition
*****************************************************************/
public class shooter {
    /****************************************************************
     * Shooter Solver Instance
    *****************************************************************/
    private Shooter_Solver _Shooter_Solver;
    private Pose3d targetPose = Constants.HopperPose;   
    /****************************************************************
     * Supplier definitions
    *****************************************************************/
    private Supplier<Pose2d> robotPoseSupplier;
    @SuppressWarnings("unused")
    private Supplier<Transform2d> robotVelocitySupplier;

    /****************************************************************
     *  Initialization Functions
    *****************************************************************/
    public shooter(Supplier<Pose2d> robotPoseSupplier, Supplier<Transform2d> robotVelocitySupplier){
        setupShooterSolver(robotPoseSupplier, robotVelocitySupplier);
    }
    public static shooter getInstance(Supplier<Pose2d> robotPoseSupplier, Supplier<Transform2d> robotVelocitySupplier){
        return new shooter(robotPoseSupplier, robotVelocitySupplier);
    }
    /****************************************************************
     * Used to change the target pose, with transform based on alliance
    *****************************************************************/
    public void change_target(Pose3d targetPose){
        // Determine the current alliance
        var alliance = DriverStation.getAlliance();
        
        // Initialize with identity transform (no change for Blue alliance)
        Transform3d allianceTransform = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        
        // Apply Red alliance coordinate transform if applicable
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            System.out.println("Red Alliance Target Transform Applied");
            allianceTransform = Constants.RED_ALLIANCE_COORDINATE_TRANSFORM;
        }
        
        // Create transform from the input target pose (assuming flat target with no rotation)
        Transform3d fieldTargetTransform3d = new Transform3d(targetPose.getTranslation(), new Rotation3d(0,0,0));
        
        // Combine alliance transform with target pose transform
        fieldTargetTransform3d = allianceTransform.plus(fieldTargetTransform3d);
        
        // Update the internal target pose with the transformed coordinates
        this.targetPose = new Pose3d(fieldTargetTransform3d.getTranslation(),fieldTargetTransform3d.getRotation());
        System.out.println(this.targetPose.getX()+", "+this.targetPose.getY()+", "+this.targetPose.getZ());
    }
    /****************************************************************
     * Supplier for target pose
    *****************************************************************/
    private Pose3d targetPoseSup(){
        return this.targetPose;
    }
    /****************************************************************
     * Function to allow or disallow shooting based on robot position. Function also takes position into account 
     * to set target pose based on where the robot is on the field.
    *****************************************************************/
    public void AllowShooting(boolean allow){
        // Determine the current alliance
        var alliance = DriverStation.getAlliance();
        boolean isRed = false;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            isRed = true;
        }
        // if within the local zone, target hopper, else target nearfield left or right based on which side of the field the robot is on
        if((!isRed &&  robotPoseSupplier.get().getX() < Constants.LOCAL_ZONE_X_LIMIT) || (isRed && (robotPoseSupplier.get().getX() > (Constants.FIELD_LENGTH - Constants.LOCAL_ZONE_X_LIMIT)))){
            this.change_target(Constants.HopperPose);
        
        }else{ // if outside local zone, shoot home nearfield based on which side of the field the robot is on
            if((!isRed && robotPoseSupplier.get().getY() > Constants.FIELD_WIDTH/2 ) || (isRed && (robotPoseSupplier.get().getY() < (Constants.FIELD_WIDTH/2)))){
                this.change_target(Constants.nearfieldLeftPose);
            }else{
                this.change_target(Constants.nearfieldRightPose);
            }
        }
           

        _Shooter_Solver.AllowShooting(allow);

    }
    /****************************************************************
     * Shooter Solver Setup
    *****************************************************************/
    @SuppressWarnings("unused")
    private void setupShooterSolver(Supplier<Pose2d> robotPoseSupplier, Supplier<Transform2d> robotVelocitySupplier){
         /****************************************************************
         * Setup Suppliers for Shooter Solver
        *****************************************************************/
        Supplier<Pose3d> targetPoseSupplier = () -> targetPoseSup();
        
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        
        // Shooter-to-robot transform supplier
        Translation3d shooterRelativePosition = new Translation3d(0.3, 0.0, 0.5);
        
        double shooterYawtoRobot = 0.0;
        
        Supplier<Transform3d> shooterTransformSupplier = () -> new Transform3d(shooterRelativePosition, new Rotation3d(0,0,Math.toRadians(shooterYawtoRobot)));
        
        double shooterMuzzleV0 = 9.5; // Muzzle velocity (m/s)
        
        /****************************************************************
         * Get Supplier for muzzle velocity from SmartDashboard
        *****************************************************************/
        SmartDashboard.putNumber("Shooter Solver Muzzle V0", shooterMuzzleV0);
        Supplier<Double> shooterMuzzleV0Supplier = () -> SmartDashboard.getNumber("Shooter Solver Muzzle V0", shooterMuzzleV0);
        
        // Create the Shooter_Solver instance
        _Shooter_Solver = Shooter_Solver.getInstance(1,
        robotPoseSupplier,
        robotVelocitySupplier,
        shooterTransformSupplier,
        targetPoseSupplier,
        shooterMuzzleV0Supplier
        );
        _Shooter_Solver.getBestSolutionStatus(); // warmup and get rid of java warnings
        
    
    }
    /****************************************************************
     * Getters for required shooter yaw and shooting allowed status
    *****************************************************************/
    public double getRobotRequiredYawRad(){
        return _Shooter_Solver.getRobotRequiredYawRad();
    }
    public boolean isShootingAllowed(){
        return _Shooter_Solver.isShootingAllowed();
    }
    
}

