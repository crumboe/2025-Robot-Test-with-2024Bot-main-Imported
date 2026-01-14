package frc.robot.subsystems.ultilities;

/*
 * ╔════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
 * ║  CAMERA SUBSYSTEM: Vision-Based Robot Localization and Sensor Fusion                                      ║
 * ╚════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
 * 
 * PURPOSE:
 * --------
 * This subsystem uses a PhotonVision camera to detect AprilTags on the field and compute the robot's absolute
 * position on the field. It acts as a critical external sensor for correcting odometry drift and improving
 * localization accuracy throughout autonomous and teleoperated modes.
 * 
 * ROLE IN THE KALMAN FILTER:
 * --------------------------
 * The Drive subsystem maintains a SwerveDrivePoseEstimator (which internally uses an Extended Kalman Filter)
 * that fuses multiple sensor inputs:
 * 
 *   1. PRIMARY STATE (Kinematics):
 *      - Swerve module velocities and positions computed from motor encoders
 *      - Gyroscope heading measurement from the IMU
 *      - These form the PREDICTION step: extrapolate pose based on motor commands
 * 
 *   2. VISION MEASUREMENT (This Camera Class):
 *      - AprilTag detection using PhotonVision processing
 *      - Absolute position computed from known AprilTag field positions
 *      - Acts as the MEASUREMENT/CORRECTION input to the Kalman filter
 * 
 * SENSOR FUSION ALGORITHM:
 * -----------------------
 * The SwerveDrivePoseEstimator uses a recursive Kalman filter equation:
 * 
 *   x̂(k+1) = A·x̂(k) + B·u(k)  [Prediction: odometry + motor commands]
 *   z(k)    = y(k) - H·x̂(k)    [Innovation: vision measurement vs predicted pose]
 *   x̂(k)    = x̂(k) + K·z(k)    [Correction: adjust prediction based on vision]
 * 
 * Where:
 *   x̂     = estimated robot pose (x, y, theta)
 *   u     = motor velocities from swerve modules
 *   z     = innovation (difference between vision and odometry)
 *   K     = Kalman gain (depends on measurement uncertainty from this camera)
 *   H     = measurement model (camera sees absolute position)
 * 
 * UNCERTAINTY MODELING:
 * --------------------
 * Camera confidence is quantified via standard deviations (stddevs) that directly impact Kalman gain:
 * 
 *   - SINGLE TAG: kSingleTagStdDevs = [2, 2, 4] meters/radians
 *     When only ONE AprilTag is visible, pose solution is ambiguous and less certain.
 *     Higher stddev → Kalman filter trusts odometry MORE than this measurement.
 * 
 *   - MULTI TAG: kMultiTagStdDevs = [0.5, 0.5, 1] meters/radians
 *     When 2+ AprilTags are visible, pose is well-constrained and highly accurate.
 *     Lower stddev → Kalman filter trusts vision MORE and corrects odometry drift faster.
 * 
 *   - TEMPORAL WEIGHTING: Further frames with more/closer tags get even lower stddev.
 * 
 * WORKFLOW:
 * ---------
 * 1. periodic(): Called every 20ms by the scheduler
 *    - Poll PhotonVision for new camera frames
 *    - If AprilTags detected: estimate robot pose and compute confidence (stddev)
 *    - Sanity check: reject outliers far from current odometry estimate
 *    - Call estConsumer.accept(pose, timestamp, stddev) to send to Drive subsystem
 * 
 * 2. Drive.periodic(): Receives vision measurements
 *    - SwerveDrivePoseEstimator.addVisionMeasurement(pose, timestamp, stddev)
 *    - Kalman filter calculates innovation and applies correction automatically
 *    - Updated pose now reflects both odometry AND vision data
 * 
 * SELF-CALIBRATION (Auto-Tuning):
 * --------------------------------
 * guessMyOwnLocation(robotPose): Periodically auto-calibrates camera mounting position
 *    - Uses odometry + vision detections to infer actual camera transform
 *    - Compensates for small installation misalignment (lens position, rotation)
 *    - Applies incremental correction to robotToCam transform (weighted 0.3 smoothing factor)
 * 
 * KEY ASSUMPTIONS:
 * ----------------
 *   • AprilTag field layout is known and properly configured in Constants
 *   • PhotonVision runs on a coprocessor (not consuming main CPU)
 *   • Camera frame rate is ~30 Hz (timestamps accurate to ±33 ms)
 *   • AprilTag IDs are unique and do not repeat across the field
 *   • No latency beyond network transmission (typically <100 ms)
 * 
 * INTEGRATION POINTS:
 * -------------------
 *   • RobotContainer: Creates Camera instance with robotToCam transform
 *   • Drive.periodic(): Calls estConsumer (callback) to feed Kalman filter
 *   • Drive.periodic(): Calls guessMyOwnLocation() for auto-calibration every N seconds
 * 
 * DEBUGGING TIPS:
 * ---------------
 *   • If odometry drifts over time: Camera stddev may be too high (filter trusts vision too little)
 *   • If motion jerks/snaps suddenly: Camera stddev may be too low (filter over-corrects on noise)
 *   • If calibration fails: Verify AprilTag field positions and camera transform are correct
 *   • Watch SmartDashboard for vision rejection rate (indicates poor measurement quality)
 */

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import java.util.List;
import java.util.Optional;

import frc.robot.Constants;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Camera extends SubsystemBase {
    // Singleton instance for camera management
    public static Camera instance;
    // User-friendly camera identifier
    public final String name;
    // PhotonVision camera interface
    private PhotonCamera camera;
    // Estimates robot pose using AprilTag detection
    private final PhotonPoseEstimator photonPoseEstimator;
    // Standard deviations for current vision measurement confidence
    private Matrix<N3, N1> curStdDevs;
    // Callback to send pose estimates to drive subsystem
    private final EstimateConsumer estConsumer;
    // Transform from robot center to camera lens
    private Transform3d robotToCam;
    // Calibrated camera transform derived from vision measurements
    private Transform3d GuessedRobotToCam;
    // Flag to pause periodic vision updates
    private boolean periodic_paused = false;
    // Supplier for current robot pose to validate vision estimates
    private Supplier<Pose2d> robotPoseGetter;
    // Tracks whether auto-calibration is active
    @SuppressWarnings("unused")
    private boolean robotPoseGuessing = false;
    // Timestamp of last camera calibration attempt
    @SuppressWarnings("unused")
    private double lastTimeRELocalized;
    // Delay interval for periodic camera calibration
    @SuppressWarnings("unused")
    private double RelocalizationRandomAdd;
    // Standard deviation for single AprilTag detections (higher uncertainty)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
    // Standard deviation for multiple AprilTag detections (lower uncertainty)
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, 1);

    public Camera(String cameraName, Transform3d robotToCam_in, EstimateConsumer estconsumer) {
        System.out.printf("befoore %s\n", cameraName);
        // Initialize PhotonVision camera connection
        this.camera = new PhotonCamera(cameraName);
        System.out.printf("in %s\n", cameraName);
        this.name = cameraName;
        this.estConsumer = estconsumer;
        // Store the physical mounting position of the camera relative to robot center
        robotToCam = robotToCam_in;
        lastTimeRELocalized = Timer.getFPGATimestamp();
        // Random delay for staggered calibration attempts
        RelocalizationRandomAdd = 15+ (Math.random()-.5) * Constants.CAMERA_RELOCALIZE_GUESS_OWN_LOC_INTERVAL/ 2 ;
        // Create pose estimator using AprilTag field layout and multi-tag strategy
        photonPoseEstimator = new PhotonPoseEstimator(Constants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);

        // Fallback strategy when primary pose estimation fails
        photonPoseEstimator.setMultiTagFallbackStrategy((PoseStrategy.LOWEST_AMBIGUITY));
    }

    public static Camera getInstance(String cameraName, Transform3d robotToCam, EstimateConsumer estConsumer) {
        // Create singleton instance on first call
        if (instance == null) {
            instance = new Camera(cameraName, robotToCam, estConsumer);
        }
        return instance;
    }

    // Check if camera has new pipeline results to process
    public boolean hasNewFrame() {
        return camera.getPipelineIndex() > 0;
    }

    // Temporarily pause vision updates while processing other critical tasks
    public void pause_periodic(boolean paused) {
        this.periodic_paused = paused;
    }
    
    // Set up the supplier to provide current robot pose for validation
    public void startGuessingOwnLoca(Supplier <Pose2d> robotPoseSup){
        this.robotPoseGetter = robotPoseSup;
    }
    @Override
    public void periodic() {
        // Skip vision processing if paused
        if (this.periodic_paused ) {
            return;
        }

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // Process all new camera frames since last update
        for (var change : camera.getAllUnreadResults()) {
            // Skip frames with no detected AprilTags
            if(!change.hasTargets()){
                continue;
            }
            // Skip frames without multi-tag detection results
            if(!change.multitagResult.isPresent()){
                continue;
            }
            // Estimate robot pose from detected AprilTags
            visionEst = photonPoseEstimator.update(change);
            // Calculate confidence level based on number and distance of visible tags
            updateEstimationStdDevs(visionEst, change.getTargets());

            // Validate and send pose to drive subsystem
            visionEst.ifPresent(
                    est -> {
                        // Adjust confidence based on tag detection quality
                        var estStdDevs = getEstimationStdDevs();
                        // Calculate difference between vision and odometry estimates
                        var err = robotPoseGetter.get().minus(est.estimatedPose.toPose2d());
                        double distErr = Math.abs(Math.hypot(err.getX(),err.getY()));
                        // Only use vision updates that are close to our current position estimate
                        if(distErr < Constants.ACCEPTABLE_CAMERA_POSE_UPDATE_DISTANCE){
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        }
                    });
            if(visionEst.isPresent()){
                break;
            }

        }
    }
    // Temporary variables for pose guessing algorithm
    private List<PhotonPipelineResult> results;
    private Pose3d RobotTransform;
    private Optional<EstimatedRobotPose> visionEst;
    private Transform3d errorTransform3d;
    
    // Auto-calibrate camera mount transform using current robot odometry and vision detections
    public void guessMyOwnLocation(Pose2d RobotPose){
        results.clear();
        results = camera.getAllUnreadResults();
        // No new frames to analyze
        if(results.isEmpty()){
            return;
        }
        // Convert robot odometry to 3D pose
        RobotTransform = new Pose3d(new Translation3d(RobotPose.getX(),RobotPose.getY(),0),new Rotation3d(0,0,RobotPose.getRotation().getRadians()));
        visionEst = Optional.empty();
        errorTransform3d = new Transform3d();
        // Analyze each camera frame for calibration
        for(var change : results){
            // Estimate pose from AprilTag detections
            visionEst = photonPoseEstimator.update(change);

            if (visionEst.isPresent()){
                // Actual vision detection found - calculate calibration error
                // Convert vision result back to camera pose in field coordinates
                Pose3d cameraPose = visionEst.get().estimatedPose.transformBy(robotToCam.inverse());
                // Determine the difference between expected and actual camera position
                GuessedRobotToCam = cameraPose.minus(RobotTransform);
                // Calculate the mounting error
                errorTransform3d = robotToCam.plus(GuessedRobotToCam.inverse());

                // Apply a portion of the error (0.3) to smoothly correct camera calibration
                robotToCam = GuessedRobotToCam.plus(errorTransform3d.times(0.3));
                photonPoseEstimator.setRobotToCameraTransform(robotToCam);

                break;
            }      

        }
    }
    // Cache for AprilTag field poses during estimation calculations
    private Optional<Pose3d> tagPose;
    
    // Calculate standard deviation for pose estimates based on detection quality
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose estimate available - use conservative uncertainty
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Analyze detection quality to adjust confidence
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;
            double avgAmbiguity = 0;
            
            // Collect statistics about detected AprilTags
            for (var tgt : targets) {
                tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                // Skip tags that aren't in the field layout
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                // Calculate distance from robot to each tag
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                // Track how ambiguous each detection is
                avgAmbiguity += tgt.poseAmbiguity;
            }

            if (numTags == 0) {
                // No valid tags found - use conservative estimate
                curStdDevs = kSingleTagStdDevs;
            } else {
                // Calculate averages
                avgDist /= numTags;
                avgAmbiguity /= numTags;
                
                // More tags detected = higher confidence (lower uncertainty)
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                    
                // Distant tags = higher uncertainty
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    
                // Apply ambiguity penalty to uncertainty
                estStdDevs = estStdDevs.times(1 + avgAmbiguity);
                curStdDevs = estStdDevs;
            }
        }
    }

    // Retrieve the current pose estimation confidence level
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // Functional interface for sending pose updates to the drive subsystem
    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
