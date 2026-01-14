package frc.robot.subsystems.ultilities;

import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Constants;
import frc.robot.commands.TunableHolonomicController;

import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.commands.PathPlannerAuto;

/** Base command for following a path */
public class FollowPathCommand103 extends Command {
    private double CostFunctionAccumulator = 0.0;
    private final Timer timer = new Timer();
    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private final TunableHolonomicController controller;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;
    private final EventScheduler eventScheduler;
    private boolean interrupted = false;
    private double pauseTimeOffset = 0.0;

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;

  /**
   * Construct a base path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current field-relative chassis speeds
   * @param output Output function that accepts field-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public FollowPathCommand103(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      TunableHolonomicController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    // Add all event scheduler requirements to this command's requirements
    var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    this.path = this.originalPath;
    // Ensure the ideal trajectory is generated
    Optional<PathPlannerTrajectory> idealTrajectory =
        this.path.getIdealTrajectory(this.robotConfig);
    idealTrajectory.ifPresent(traj -> this.trajectory = traj);
  }

  @Override
  public void initialize() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if (path.getIdealStartingState() != null) {
      // Check if we match the ideal starting state
      boolean idealVelocity =
          Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation =
          !robotConfig.isHolonomic
              || Math.abs(
                      currentPose
                          .getRotation()
                          .minus(path.getIdealStartingState().rotation())
                          .getDegrees())
                  <= 30.0;
      if (idealVelocity && idealRotation) {
        // We can use the ideal trajectory
        trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
      } else {
        // We need to regenerate
        // System.out.printf("%f,%f,%f\n\n",currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond,currentPose.getRotation().getDegrees());
        trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
      }
    } else {
      // No ideal starting state, generate the trajectory
      trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
    }

    PathPlannerAuto.setCurrentTrajectory(trajectory);
    PathPlannerAuto.currentPathName = originalPath.name;

    PathPlannerLogging.logActivePath(path);
    PPLibTelemetry.setCurrentPath(path);

    eventScheduler.initialize(trajectory);

    timer.reset();
    timer.start();
  }

  @Override
  // TODO Test Functionallity
  public void execute() {
    double currentTime = timer.get();
    Pose2d currentPose = poseSupplier.get();
    var targetState = trajectory.sample(currentTime-pauseTimeOffset);
    Pose2d targetPose = targetState.pose;
    double DistanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    
    
    // if (!controller.isHolonomic() && path.isReversed()) {
    
    // }
    
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    ChassisSpeeds targetSpeeds = controller.calculateFieldSpeeds(currentPose, targetState);
    
    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    Transform2d transform = targetPose.minus(currentPose);
    // double anlge =transform.getTranslation().getAngle().getRadians(); 
    // double distanceMod = currentVel; 
    
    CostFunctionAccumulator += 1.5* Math.pow(DistanceError, 2) 
                             + 1.5* Math.pow(transform.getRotation().getRadians(), 2)
                             + 1* Math.pow(currentVel - targetState.linearVelocity, 2);
                                     
    // System.out.printf("%f %f\n",currentVel, targetState.linearVelocity);
    // if(DistanceError > Constants.PP103DistErrorLim){ // if farther than limit from point target, do not move to the next target, instead drive towards it
        
    //     timer.stop(); // stop the trajectory and event timer
    //     double omega = .1*transform.getRotation().getRadians();
        
    //     // create new vector towards target point with same speed as current speed
    //     targetSpeeds = new ChassisSpeeds(distanceMod*Math.cos(anlge),
    //                                      distanceMod*Math.sin(anlge),
    //                                      omega);                            
    // }else{
    //     if(!timer.isRunning()){
    //         timer.start();
    //     }
    // }

    // PPLibTelemetry.setCurrentPose(currentPose);
    // PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(targetState.pose);
    // PathPlannerLogging.logTargetPose(targetState.pose);

    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);

    output.accept(targetSpeeds, targetState.feedforwards);

    eventScheduler.execute(currentTime);
  }

  @Override
  public boolean isFinished() {
    return this.interrupted || timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    this.interrupted = interrupted;
    timer.stop();
    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);

    // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
      output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    // PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }
  public double getCostFunctionAccumulator(){
    return CostFunctionAccumulator;
  }
  /**
   * Create a command to warmup on-the-fly generation, replanning, and the path following command
   *
   * @return Path following warmup command
   */
  public static Command warmupCommand() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(0.0, 0.0, Rotation2d.kZero), new Pose2d(6.0, 6.0, Rotation2d.kZero));
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(4.0, 4.0, 4.0, 4.0),
            new IdealStartingState(0.0, Rotation2d.kZero),
            new GoalEndState(0.0, Rotation2d.kCW_90deg));

    return new FollowPathCommand103(
            path,
            () -> Pose2d.kZero,
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new TunableHolonomicController(
                5.0, 0.0, 0.0,   // x PID
                5.0, 0.0, 0.0,   // y PID
                5.0, 0.0, 0.0,   // theta PID
                Math.toRadians(720), Math.toRadians(720) // rot constraints (example)
            ),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55),
            () -> true)
        .andThen(Commands.print("[PathPlanner] FollowPathCommand103 finished warmup"))
        .ignoringDisable(true);
  }
}
