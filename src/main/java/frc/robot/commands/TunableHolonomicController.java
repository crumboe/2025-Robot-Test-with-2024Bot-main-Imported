package frc.robot.commands;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableHolonomicController implements PathFollowingController {
  // Expose these to Shuffleboard/NT and change kP/kI/kD mid-runtime
  private final PIDController xPid;
  private final PIDController yPid;
  private final ProfiledPIDController thetaPid;

  public TunableHolonomicController(double kPx, double kIx, double kDx,
                                    double kPy, double kIy, double kDy,
                                    double kPt, double kIt, double kDt,
                                    double maxOmega, double maxAlpha) {
    xPid = new PIDController(kPx, kIx, kDx);
    yPid = new PIDController(kPy, kIy, kDy);
    thetaPid = new ProfiledPIDController(
        kPt, kIt, kDt, new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
    // Continuous angle (-π..π) is important
    thetaPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    xPid.reset();
    yPid.reset();
    thetaPid.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public boolean isHolonomic() {
    return true;
  }

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, PathPlannerTrajectoryState targetState) {

    // Feedforward from the path (field-relative)
    var ff = targetState.fieldSpeeds; // vx, vy (m/s, field frame) and omega (rad/s)
    var ref = targetState.pose;       // field-relative reference pose at this time

    // Feedback terms computed in field frame
    double vxFbField = xPid.calculate(currentPose.getX(), ref.getX());
    double vyFbField = yPid.calculate(currentPose.getY(), ref.getY());
    double omegaFb   = thetaPid.calculate(
        currentPose.getRotation().getRadians(),
        ref.getRotation().getRadians());

    // Sum FF + FB in field frame, then convert to robot-relative speeds
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        ff.vxMetersPerSecond + vxFbField,
        ff.vyMetersPerSecond + vyFbField,
        ff.omegaRadiansPerSecond + omegaFb,
        currentPose.getRotation());
  }
  
  public ChassisSpeeds calculateFieldSpeeds(
      Pose2d currentPose, PathPlannerTrajectoryState targetState) {

    // Feedforward from the path (field-relative)
    var ff = targetState.fieldSpeeds; // vx, vy (m/s, field frame) and omega (rad/s)
    var ref = targetState.pose;       // field-relative reference pose at this time

    // Feedback terms computed in field frame
    // System.out.printf("%f %f %f %f\n",currentPose.getX(), ref.getX(),currentPose.getY(), ref.getY());
    double vxFbField = xPid.calculate(currentPose.getX(), ref.getX());
    double vyFbField = yPid.calculate(currentPose.getY(), ref.getY());
    double omegaFb   = thetaPid.calculate(
        currentPose.getRotation().getRadians(),
        ref.getRotation().getRadians());

    // Sum FF + FB in field frame, then LEAVE IN FIELD FRAME
    return new ChassisSpeeds(  vxFbField + ff.vxMetersPerSecond , 
                               vyFbField + ff.vyMetersPerSecond , 
                                 omegaFb + ff.omegaRadiansPerSecond );
  }

  // Optional helpers so you can tweak at runtime
  public void setTranslationGains(double kP, double kI, double kD) {
    xPid.setPID(kP, kI, kD);
    yPid.setPID(kP, kI, kD);
  }
  public void setRotationGains(double kP, double kI, double kD) {
    thetaPid.setPID(kP, kI, kD);
  }
}
