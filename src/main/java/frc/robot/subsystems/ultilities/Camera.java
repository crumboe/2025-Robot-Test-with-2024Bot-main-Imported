package frc.robot.subsystems.ultilities;

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
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static Camera instance;
    public final String name;
    private PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private Transform3d robotToCam;
    private Transform3d GuessedRobotToCam;
    private boolean periodic_paused = false;
    private Supplier<Pose2d> robotPoseGetter;
    private boolean robotPoseGuessing = false;
    private double lastTimeRELocalized;
    private double RelocalizationRandomAdd;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, 1);

    public Camera(String cameraName, Transform3d robotToCam_in, EstimateConsumer estconsumer) {
        System.out.printf("befoore %s\n", cameraName);
        this.camera = new PhotonCamera(cameraName);
        // PhotonCamera.setVersionCheckEnabled(false);
        System.out.printf("in %s\n", cameraName);
        this.name = cameraName;
        this.estConsumer = estconsumer;
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        robotToCam = robotToCam_in;
        lastTimeRELocalized = Timer.getFPGATimestamp();
        RelocalizationRandomAdd = 15+ (Math.random()-.5) * Constants.CAMERA_RELOCALIZE_GUESS_OWN_LOC_INTERVAL/ 2 ; // delay that beginning of this to allow robot pose to stabilize
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(Constants.aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);

        photonPoseEstimator.setMultiTagFallbackStrategy((PoseStrategy.LOWEST_AMBIGUITY));
    }

    public static Camera getInstance(String cameraName, Transform3d robotToCam, EstimateConsumer estConsumer) {
        if (instance == null) {
            instance = new Camera(cameraName, robotToCam, estConsumer);
        }
        return instance;
    }

    public boolean hasNewFrame() {
        return camera.getPipelineIndex() > 0;
    }

    public void pause_periodic(boolean paused) {
        this.periodic_paused = paused;
    }
    public void startGuessingOwnLoca(Supplier <Pose2d> robotPoseSup){
        this.robotPoseGetter = robotPoseSup;
        // robotPoseGuessing = true;
    }
    @Override
    public void periodic() {
        // if (this.robotPoseGuessing){
        //     double currentTime = Timer.getFPGATimestamp();
        //     if(currentTime - lastTimeRELocalized > (Constants.CAMERA_RELOCALIZE_GUESS_OWN_LOC_INTERVAL) + RelocalizationRandomAdd){ // add some randomness to avoid sync with other systems
        //         lastTimeRELocalized = currentTime;
        //         RelocalizationRandomAdd = (Math.random()-.5) * Constants.CAMERA_RELOCALIZE_GUESS_OWN_LOC_INTERVAL/ 2 ;
        //         guessMyOwnLocation(this.robotPoseGetter.get());
        //     }
        // }
        // if(!this.camera){
        //     camera.close();
        //     camera = new PhotonCamera(this.name);
        // }

        if (this.periodic_paused ) {
            return;
        }

        // System.out.print("I'm Here");
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            if(!change.hasTargets()){
                continue;
            }
            if(!change.multitagResult.isPresent()){
                continue;
            }
            visionEst = photonPoseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        var err = robotPoseGetter.get().minus(est.estimatedPose.toPose2d());
                        double distErr = Math.abs(Math.hypot(err.getX(),err.getY()));
                        if(distErr < Constants.ACCEPTABLE_CAMERA_POSE_UPDATE_DISTANCE){ // reject new poses that are too far from the robot pose
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        }
                    });
            if(visionEst.isPresent()){
                break;
            }

        }
    }
    private List<PhotonPipelineResult> results; private Pose3d RobotTransform;private Optional<EstimatedRobotPose> visionEst; private Transform3d errorTransform3d;
    public void guessMyOwnLocation(Pose2d RobotPose){
        results.clear();
        results = camera.getAllUnreadResults();
        if(results.isEmpty()){
            return;
        }
        RobotTransform = new Pose3d(new Translation3d(RobotPose.getX(),RobotPose.getY(),0),new Rotation3d(0,0,RobotPose.getRotation().getRadians()));
        visionEst = Optional.empty();
        errorTransform3d = new Transform3d();
        for(var change : results){
            
            visionEst = photonPoseEstimator.update(change);
            // updateEstimationStdDevs(visionEst, change.getTargets());

            if (visionEst.isPresent()){
                //we have a real vision estimate, no need to guess
                Pose3d cameraPose = visionEst.get().estimatedPose.transformBy(robotToCam.inverse()); // subtract old transform to get detected camera pose
                GuessedRobotToCam = cameraPose.minus(RobotTransform); //get difference between expected to find new transform
                errorTransform3d = robotToCam.plus(GuessedRobotToCam.inverse());

                robotToCam = GuessedRobotToCam.plus(errorTransform3d.times(0.3)); // apply a portion of the error to smooth out changes
                photonPoseEstimator.setRobotToCameraTransform(robotToCam);


                break;
            }      

        }
            
    
    }
    private Optional<Pose3d> tagPose;
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;
            double avgAmbiguity =0;
            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            
            for (var tgt : targets) {
                tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                        avgAmbiguity += tgt.poseAmbiguity;
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                avgAmbiguity /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                estStdDevs = estStdDevs.times(1+avgAmbiguity);
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
