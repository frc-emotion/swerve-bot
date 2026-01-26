package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.VisionConstants;

/**
 * Vision subsystem that manages multiple PhotonVision cameras for AprilTag detection
 * and robot pose estimation.
 *
 * Based on PhotonVision documentation:
 * https://docs.photonvision.org/en/v2026.0.1-beta/docs/programming/photonlib/robot-pose-estimator.html
 */
public class Vision extends SubsystemBase {

    // Telemetry throttling - update SmartDashboard every N cycles (10 = 5Hz at 50Hz loop)
    private static final int TELEMETRY_UPDATE_INTERVAL = 10;

    // Cameras
    private final PhotonCamera cameraRight;
    private final PhotonCamera cameraLeft;

    // Pose estimators for each camera
    private final PhotonPoseEstimator poseEstimatorRight;
    private final PhotonPoseEstimator poseEstimatorLeft;

    // Latest results cache
    private PhotonPipelineResult latestResultRight;
    private PhotonPipelineResult latestResultLeft;

    // Latest estimated poses
    private Optional<EstimatedRobotPose> latestEstimateRight = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimateLeft = Optional.empty();

    // Standard deviations for the latest estimates
    private Matrix<N3, N1> currentStdDevsRight = VisionConstants.SINGLE_TAG_STD_DEVS;
    private Matrix<N3, N1> currentStdDevsLeft = VisionConstants.SINGLE_TAG_STD_DEVS;

    // Diagnostic counters
    private int periodicCallCount = 0;
    private int rightResultCount = 0;
    private int leftResultCount = 0;

    // Vision enable/disable flag for testing
    private boolean visionEnabled = true;

    // Debug data cache (updated every cycle, logged throttled)
    private int debugRightSeenTagId = -1;
    private int debugRightNumTargets = 0;
    private boolean debugRightTagInLayout = false;
    private double debugRightTagFieldX = 0;
    private double debugRightTagFieldY = 0;
    private boolean debugRightEstimatePresent = false;
    private double debugRightRawX = 0;
    private double debugRightRawY = 0;
    private double debugRightRawZ = 0;
    private boolean debugRightPassedValidation = false;
    private boolean debugRightUsingFallback = false;
    private double debugRightCamToTargetX = 0;
    private double debugRightCamToTargetY = 0;
    private double debugRightCamToTargetZ = 0;
    private double debugRightFallbackX = 0;
    private double debugRightFallbackY = 0;
    private double debugRightFallbackRot = 0;

    private int debugLeftSeenTagId = -1;
    private int debugLeftNumTargets = 0;
    private boolean debugLeftTagInLayout = false;
    private boolean debugLeftEstimatePresent = false;
    private double debugLeftRawX = 0;
    private double debugLeftRawY = 0;
    private double debugLeftRawZ = 0;
    private boolean debugLeftPassedValidation = false;
    private boolean debugLeftUsingFallback = false;
    private double debugLeftFallbackX = 0;
    private double debugLeftFallbackY = 0;
    private double debugLeftFallbackRot = 0;

    private boolean debugRightHasTargets = false;
    private boolean debugLeftHasTargets = false;

    /**
     * Creates a new Vision subsystem with two cameras.
     */
    public Vision() {
        // Initialize cameras with names matching PhotonVision UI
        cameraRight = new PhotonCamera(VisionConstants.CAMERA_NAME_RIGHT);
        cameraLeft = new PhotonCamera(VisionConstants.CAMERA_NAME_LEFT);

        // Check for valid field layout before creating pose estimators
        if (VisionConstants.TAG_LAYOUT == null) {
            System.err.println("WARNING: AprilTag field layout is null! Vision pose estimation will be disabled.");
            poseEstimatorRight = null;
            poseEstimatorLeft = null;
            return;
        }

        // Initialize pose estimators using MULTI_TAG_PNP_ON_COPROCESSOR strategy
        // This is the recommended strategy per PhotonVision docs for best accuracy
        poseEstimatorRight = new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_CAM_RIGHT
        );

        poseEstimatorLeft = new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_CAM_LEFT
        );

        // Set fallback strategy for when only one tag is visible
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        periodicCallCount++;

        // Process all unread results from both cameras (always runs - this is the critical path)
        updateCameraRight();
        updateCameraLeft();

        // Log telemetry at reduced rate to avoid loop overruns
        if (periodicCallCount % TELEMETRY_UPDATE_INTERVAL == 0) {
            logTelemetry();
            logDebugTelemetry();
        }
    }

    /**
     * Updates pose estimates from the right camera.
     */
    private void updateCameraRight() {
        latestEstimateRight = Optional.empty();
        debugRightHasTargets = false;

        // Skip if pose estimator wasn't initialized (no field layout)
        if (poseEstimatorRight == null) {
            return;
        }

        var results = cameraRight.getAllUnreadResults();
        rightResultCount += results.size();

        for (PhotonPipelineResult result : results) {
            latestResultRight = result;
            debugRightHasTargets = result.hasTargets();

            // Skip if no targets
            if (!result.hasTargets()) {
                continue;
            }

            // Debug: Show what tag IDs we're seeing
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            int tagId = bestTarget.getFiducialId();
            debugRightSeenTagId = tagId;
            debugRightNumTargets = result.getTargets().size();

            // Debug: Check if this tag ID exists in field layout
            var tagPoseOpt = VisionConstants.TAG_LAYOUT.getTagPose(tagId);
            debugRightTagInLayout = tagPoseOpt.isPresent();

            if (tagPoseOpt.isPresent()) {
                Pose3d tagPose = tagPoseOpt.get();
                debugRightTagFieldX = tagPose.getX();
                debugRightTagFieldY = tagPose.getY();
            }

            // Update the pose estimator with this result
            Optional<EstimatedRobotPose> estimate = poseEstimatorRight.update(result);
            debugRightEstimatePresent = estimate.isPresent();

            if (estimate.isPresent()) {
                Pose3d pose = estimate.get().estimatedPose;
                debugRightRawX = pose.getX();
                debugRightRawY = pose.getY();
                debugRightRawZ = pose.getZ();

                // Validate the estimate before accepting
                boolean valid = isValidEstimate(estimate.get(), result.getTargets());
                debugRightPassedValidation = valid;

                if (valid) {
                    latestEstimateRight = estimate;
                    currentStdDevsRight = calculateStdDevs(estimate.get(), result.getTargets());
                }
                debugRightUsingFallback = false;
            } else {
                // FALLBACK: If estimator fails but we have a tag in the layout, calculate manually
                if (tagPoseOpt.isPresent()) {
                    Transform3d camToTarget = bestTarget.getBestCameraToTarget();
                    debugRightCamToTargetX = camToTarget.getX();
                    debugRightCamToTargetY = camToTarget.getY();
                    debugRightCamToTargetZ = camToTarget.getZ();

                    // Calculate robot pose: tagPose - camToTarget - robotToCam
                    Pose3d tagPose = tagPoseOpt.get();
                    Pose3d cameraPose = tagPose.transformBy(camToTarget.inverse());
                    Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAM_RIGHT.inverse());

                    debugRightFallbackX = robotPose.getX();
                    debugRightFallbackY = robotPose.getY();
                    debugRightFallbackRot = robotPose.toPose2d().getRotation().getDegrees();

                    // Create a manual estimate
                    latestEstimateRight = Optional.of(new EstimatedRobotPose(
                        robotPose,
                        result.getTimestampSeconds(),
                        result.getTargets(),
                        PoseStrategy.LOWEST_AMBIGUITY
                    ));
                    currentStdDevsRight = VisionConstants.SINGLE_TAG_STD_DEVS;
                    debugRightUsingFallback = true;
                } else {
                    debugRightUsingFallback = false;
                }
            }
        }
    }

    /**
     * Updates pose estimates from the left camera.
     */
    private void updateCameraLeft() {
        latestEstimateLeft = Optional.empty();
        debugLeftHasTargets = false;

        // Skip if pose estimator wasn't initialized (no field layout)
        if (poseEstimatorLeft == null) {
            return;
        }

        var results = cameraLeft.getAllUnreadResults();
        leftResultCount += results.size();

        for (PhotonPipelineResult result : results) {
            latestResultLeft = result;
            debugLeftHasTargets = result.hasTargets();

            // Skip if no targets
            if (!result.hasTargets()) {
                continue;
            }

            // Debug: Show what tag IDs we're seeing
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            int tagId = bestTarget.getFiducialId();
            debugLeftSeenTagId = tagId;
            debugLeftNumTargets = result.getTargets().size();

            // Debug: Check if this tag ID exists in field layout
            var tagPoseOpt = VisionConstants.TAG_LAYOUT.getTagPose(tagId);
            debugLeftTagInLayout = tagPoseOpt.isPresent();

            // Update the pose estimator with this result
            Optional<EstimatedRobotPose> estimate = poseEstimatorLeft.update(result);
            debugLeftEstimatePresent = estimate.isPresent();

            if (estimate.isPresent()) {
                Pose3d pose = estimate.get().estimatedPose;
                debugLeftRawX = pose.getX();
                debugLeftRawY = pose.getY();
                debugLeftRawZ = pose.getZ();

                // Validate the estimate before accepting
                boolean valid = isValidEstimate(estimate.get(), result.getTargets());
                debugLeftPassedValidation = valid;

                if (valid) {
                    latestEstimateLeft = estimate;
                    currentStdDevsLeft = calculateStdDevs(estimate.get(), result.getTargets());
                }
                debugLeftUsingFallback = false;
            } else {
                // FALLBACK: If estimator fails but we have a tag in the layout, calculate manually
                if (tagPoseOpt.isPresent()) {
                    Transform3d camToTarget = bestTarget.getBestCameraToTarget();

                    // Calculate robot pose: tagPose - camToTarget - robotToCam
                    Pose3d tagPose = tagPoseOpt.get();
                    Pose3d cameraPose = tagPose.transformBy(camToTarget.inverse());
                    Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAM_LEFT.inverse());

                    debugLeftFallbackX = robotPose.getX();
                    debugLeftFallbackY = robotPose.getY();
                    debugLeftFallbackRot = robotPose.toPose2d().getRotation().getDegrees();

                    // Create a manual estimate
                    latestEstimateLeft = Optional.of(new EstimatedRobotPose(
                        robotPose,
                        result.getTimestampSeconds(),
                        result.getTargets(),
                        PoseStrategy.LOWEST_AMBIGUITY
                    ));
                    currentStdDevsLeft = VisionConstants.SINGLE_TAG_STD_DEVS;
                    debugLeftUsingFallback = true;
                } else {
                    debugLeftUsingFallback = false;
                }
            }
        }
    }

    /**
     * Validates an estimated pose to filter out bad measurements.
     *
     * @param estimate The estimated pose
     * @param targets The targets used in the estimate
     * @return true if the estimate should be trusted
     */
    private boolean isValidEstimate(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        // Check if pose is within field bounds (with some margin)
        Pose3d pose = estimate.estimatedPose;
        if (pose.getX() < -1 || pose.getX() > 17 ||
            pose.getY() < -1 || pose.getY() > 9 ||
            pose.getZ() < -0.5 || pose.getZ() > 1.5) {
            return false;
        }

        // For single-tag estimates, check ambiguity
        if (targets.size() == 1) {
            PhotonTrackedTarget target = targets.get(0);
            if (target.getPoseAmbiguity() > VisionConstants.MAX_AMBIGUITY) {
                return false;
            }

            // Check distance - single tags are unreliable at long range
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distance > VisionConstants.MAX_VISION_DISTANCE) {
                return false;
            }
        }

        return true;
    }

    /**
     * Calculates dynamic standard deviations based on target quality.
     * Uses the approach recommended by FRC teams for optimal pose estimation.
     *
     * @param estimate The estimated pose
     * @param targets The targets used in the estimate
     * @return Standard deviations [x, y, theta]
     */
    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        int numTargets = targets.size();

        // Multi-tag estimates are much more reliable
        if (numTargets >= VisionConstants.MIN_MULTI_TAG_TARGETS) {
            return VisionConstants.MULTI_TAG_STD_DEVS;
        }

        // Single tag - scale std devs by distance
        if (numTargets == 1) {
            PhotonTrackedTarget target = targets.get(0);
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = target.getPoseAmbiguity();

            // Scale factor increases with distance and ambiguity
            // At 1m with 0 ambiguity, factor = 1
            // At 4m with 0.2 ambiguity, factor = 4 * (1 + 0.2) = 4.8
            double scaleFactor = distance * (1 + ambiguity * 5);

            return VecBuilder.fill(
                VisionConstants.SINGLE_TAG_STD_DEVS.get(0, 0) * scaleFactor,
                VisionConstants.SINGLE_TAG_STD_DEVS.get(1, 0) * scaleFactor,
                VisionConstants.SINGLE_TAG_STD_DEVS.get(2, 0) * scaleFactor
            );
        }

        // Fallback
        return VisionConstants.SINGLE_TAG_STD_DEVS;
    }

    /**
     * Gets the latest estimated pose from the right camera.
     * @return Optional containing the estimated pose, or empty if no valid estimate or vision disabled
     */
    public Optional<EstimatedRobotPose> getEstimatedPoseRight() {
        if (!visionEnabled) {
            return Optional.empty();
        }
        return latestEstimateRight;
    }

    /**
     * Gets the latest estimated pose from the left camera.
     * @return Optional containing the estimated pose, or empty if no valid estimate or vision disabled
     */
    public Optional<EstimatedRobotPose> getEstimatedPoseLeft() {
        if (!visionEnabled) {
            return Optional.empty();
        }
        return latestEstimateLeft;
    }

    /**
     * Gets the standard deviations for the right camera's latest estimate.
     */
    public Matrix<N3, N1> getStdDevsRight() {
        return currentStdDevsRight;
    }

    /**
     * Gets the standard deviations for the left camera's latest estimate.
     */
    public Matrix<N3, N1> getStdDevsLeft() {
        return currentStdDevsLeft;
    }

    /**
     * Gets the latest pipeline result from the right camera.
     */
    public PhotonPipelineResult getLatestResultRight() {
        return latestResultRight;
    }

    /**
     * Gets the latest pipeline result from the left camera.
     */
    public PhotonPipelineResult getLatestResultLeft() {
        return latestResultLeft;
    }

    /**
     * Enum to identify which camera detected a target.
     */
    public enum CameraSource {
        RIGHT,
        LEFT,
        NONE
    }

    /**
     * Result containing a target and which camera saw it.
     */
    public record TargetWithSource(PhotonTrackedTarget target, CameraSource source) {}

    /**
     * Gets the best target from either camera (prefers lowest ambiguity).
     * @return Optional containing the best target, or empty if no targets visible
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        return getBestTargetWithSource().map(TargetWithSource::target);
    }

    /**
     * Gets the best target along with which camera detected it.
     * @return Optional containing target and camera source, or empty if no targets visible
     */
    public Optional<TargetWithSource> getBestTargetWithSource() {
        PhotonTrackedTarget bestTarget = null;
        double bestAmbiguity = Double.MAX_VALUE;
        CameraSource source = CameraSource.NONE;

        // Check right camera
        if (latestResultRight != null && latestResultRight.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultRight.getTargets()) {
                if (target.getPoseAmbiguity() < bestAmbiguity) {
                    bestAmbiguity = target.getPoseAmbiguity();
                    bestTarget = target;
                    source = CameraSource.RIGHT;
                }
            }
        }

        // Check left camera
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultLeft.getTargets()) {
                if (target.getPoseAmbiguity() < bestAmbiguity) {
                    bestAmbiguity = target.getPoseAmbiguity();
                    bestTarget = target;
                    source = CameraSource.LEFT;
                }
            }
        }

        if (bestTarget != null) {
            return Optional.of(new TargetWithSource(bestTarget, source));
        }
        return Optional.empty();
    }

    /**
     * Gets a specific target by its AprilTag ID from either camera.
     * @param tagId The fiducial ID to search for
     * @return Optional containing the target, or empty if not found
     */
    public Optional<PhotonTrackedTarget> getTargetById(int tagId) {
        // Check right camera
        if (latestResultRight != null && latestResultRight.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultRight.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target);
                }
            }
        }

        // Check left camera
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultLeft.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target);
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Checks if any target is currently visible on either camera.
     */
    public boolean hasTargets() {
        boolean rightHas = latestResultRight != null && latestResultRight.hasTargets();
        boolean leftHas = latestResultLeft != null && latestResultLeft.hasTargets();
        return rightHas || leftHas;
    }

    /**
     * Gets the yaw to the best visible target (for aiming).
     * @return yaw in degrees (positive = target is to the left)
     */
    public double getBestTargetYaw() {
        Optional<PhotonTrackedTarget> target = getBestTarget();
        return target.map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }

    /**
     * Gets the pitch to the best visible target (for distance calculation).
     * @return pitch in degrees (positive = target is above camera)
     */
    public double getBestTargetPitch() {
        Optional<PhotonTrackedTarget> target = getBestTarget();
        return target.map(PhotonTrackedTarget::getPitch).orElse(0.0);
    }

    /**
     * Gets the area of the best visible target (0-100, percentage of image).
     */
    public double getBestTargetArea() {
        Optional<PhotonTrackedTarget> target = getBestTarget();
        return target.map(PhotonTrackedTarget::getArea).orElse(0.0);
    }

    /**
     * Gets the 3D transform from camera to the best target.
     */
    public Optional<Transform3d> getBestCameraToTarget() {
        Optional<PhotonTrackedTarget> target = getBestTarget();
        return target.map(PhotonTrackedTarget::getBestCameraToTarget);
    }

    /**
     * Sets the reference pose for CLOSEST_TO_REFERENCE_POSE strategy.
     * Call this with the current odometry pose for best results.
     */
    public void setReferencePose(Pose2d pose) {
        if (poseEstimatorRight != null) {
            poseEstimatorRight.setReferencePose(pose);
        }
        if (poseEstimatorLeft != null) {
            poseEstimatorLeft.setReferencePose(pose);
        }
    }

    /**
     * Enables or disables driver mode on both cameras.
     * Driver mode shows an unprocessed camera feed for driving.
     */
    public void setDriverMode(boolean enabled) {
        cameraRight.setDriverMode(enabled);
        cameraLeft.setDriverMode(enabled);
    }

    /**
     * Sets the pipeline index on both cameras.
     * @param index Pipeline index (0-based)
     */
    public void setPipelineIndex(int index) {
        cameraRight.setPipelineIndex(index);
        cameraLeft.setPipelineIndex(index);
    }

    /**
     * Logs essential vision telemetry to SmartDashboard (throttled).
     */
    private void logTelemetry() {
        // Vision enabled status
        SmartDashboard.putBoolean("Vision/Enabled", visionEnabled);

        // Right camera status
        boolean rightConnected = cameraRight.isConnected();
        boolean rightHasTargets = latestResultRight != null && latestResultRight.hasTargets();
        SmartDashboard.putBoolean("Vision/Right/Connected", rightConnected);
        SmartDashboard.putBoolean("Vision/Right/HasTargets", rightHasTargets);

        if (rightHasTargets) {
            SmartDashboard.putNumber("Vision/Right/NumTargets", latestResultRight.getTargets().size());
            SmartDashboard.putNumber("Vision/Right/BestYaw", latestResultRight.getBestTarget().getYaw());
        }

        // Left camera status
        boolean leftConnected = cameraLeft.isConnected();
        boolean leftHasTargets = latestResultLeft != null && latestResultLeft.hasTargets();
        SmartDashboard.putBoolean("Vision/Left/Connected", leftConnected);
        SmartDashboard.putBoolean("Vision/Left/HasTargets", leftHasTargets);

        if (leftHasTargets) {
            SmartDashboard.putNumber("Vision/Left/NumTargets", latestResultLeft.getTargets().size());
            SmartDashboard.putNumber("Vision/Left/BestYaw", latestResultLeft.getBestTarget().getYaw());
        }

        // Overall best target
        Optional<PhotonTrackedTarget> best = getBestTarget();
        SmartDashboard.putBoolean("Vision/HasTarget", best.isPresent());
        if (best.isPresent()) {
            SmartDashboard.putNumber("Vision/BestTarget/ID", best.get().getFiducialId());
            SmartDashboard.putNumber("Vision/BestTarget/Yaw", best.get().getYaw());
            SmartDashboard.putNumber("Vision/BestTarget/Pitch", best.get().getPitch());
            SmartDashboard.putNumber("Vision/BestTarget/Area", best.get().getArea());
            SmartDashboard.putNumber("Vision/BestTarget/Ambiguity", best.get().getPoseAmbiguity());
        }

        // Pose estimates
        latestEstimateRight.ifPresent(est -> {
            Pose2d pose = est.estimatedPose.toPose2d();
            SmartDashboard.putNumber("Vision/Right/EstX", pose.getX());
            SmartDashboard.putNumber("Vision/Right/EstY", pose.getY());
            SmartDashboard.putNumber("Vision/Right/EstRot", pose.getRotation().getDegrees());
        });

        latestEstimateLeft.ifPresent(est -> {
            Pose2d pose = est.estimatedPose.toPose2d();
            SmartDashboard.putNumber("Vision/Left/EstX", pose.getX());
            SmartDashboard.putNumber("Vision/Left/EstY", pose.getY());
            SmartDashboard.putNumber("Vision/Left/EstRot", pose.getRotation().getDegrees());
        });
    }

    /**
     * Logs detailed debug telemetry to SmartDashboard (throttled).
     * All the detailed diagnostic info lives here.
     */
    private void logDebugTelemetry() {
        // Heartbeat and counters
        SmartDashboard.putNumber("Vision/Debug/PeriodicCount", periodicCallCount);
        SmartDashboard.putNumber("Vision/Debug/RightTotalResults", rightResultCount);
        SmartDashboard.putNumber("Vision/Debug/LeftTotalResults", leftResultCount);

        // Connection status
        SmartDashboard.putBoolean("Vision/Debug/RightCameraConnected", cameraRight.isConnected());
        SmartDashboard.putBoolean("Vision/Debug/LeftCameraConnected", cameraLeft.isConnected());

        // Right camera debug
        SmartDashboard.putBoolean("Vision/Debug/RightHasTargets", debugRightHasTargets);
        SmartDashboard.putNumber("Vision/Right/Debug/SeenTagID", debugRightSeenTagId);
        SmartDashboard.putNumber("Vision/Right/Debug/NumTargetsInResult", debugRightNumTargets);
        SmartDashboard.putBoolean("Vision/Right/Debug/TagInLayout", debugRightTagInLayout);
        SmartDashboard.putNumber("Vision/Right/Debug/TagFieldX", debugRightTagFieldX);
        SmartDashboard.putNumber("Vision/Right/Debug/TagFieldY", debugRightTagFieldY);
        SmartDashboard.putBoolean("Vision/Right/Debug/EstimatePresent", debugRightEstimatePresent);
        SmartDashboard.putNumber("Vision/Right/Debug/RawX", debugRightRawX);
        SmartDashboard.putNumber("Vision/Right/Debug/RawY", debugRightRawY);
        SmartDashboard.putNumber("Vision/Right/Debug/RawZ", debugRightRawZ);
        SmartDashboard.putBoolean("Vision/Right/Debug/PassedValidation", debugRightPassedValidation);
        SmartDashboard.putBoolean("Vision/Right/Debug/UsingFallback", debugRightUsingFallback);
        SmartDashboard.putNumber("Vision/Right/Debug/CamToTargetX", debugRightCamToTargetX);
        SmartDashboard.putNumber("Vision/Right/Debug/CamToTargetY", debugRightCamToTargetY);
        SmartDashboard.putNumber("Vision/Right/Debug/CamToTargetZ", debugRightCamToTargetZ);
        SmartDashboard.putNumber("Vision/Right/Debug/FallbackX", debugRightFallbackX);
        SmartDashboard.putNumber("Vision/Right/Debug/FallbackY", debugRightFallbackY);
        SmartDashboard.putNumber("Vision/Right/Debug/FallbackRot", debugRightFallbackRot);

        // Left camera debug
        SmartDashboard.putBoolean("Vision/Debug/LeftHasTargets", debugLeftHasTargets);
        SmartDashboard.putNumber("Vision/Left/Debug/SeenTagID", debugLeftSeenTagId);
        SmartDashboard.putNumber("Vision/Left/Debug/NumTargetsInResult", debugLeftNumTargets);
        SmartDashboard.putBoolean("Vision/Left/Debug/TagInLayout", debugLeftTagInLayout);
        SmartDashboard.putBoolean("Vision/Left/Debug/EstimatePresent", debugLeftEstimatePresent);
        SmartDashboard.putNumber("Vision/Left/Debug/RawX", debugLeftRawX);
        SmartDashboard.putNumber("Vision/Left/Debug/RawY", debugLeftRawY);
        SmartDashboard.putNumber("Vision/Left/Debug/RawZ", debugLeftRawZ);
        SmartDashboard.putBoolean("Vision/Left/Debug/PassedValidation", debugLeftPassedValidation);
        SmartDashboard.putBoolean("Vision/Left/Debug/UsingFallback", debugLeftUsingFallback);
        SmartDashboard.putNumber("Vision/Left/Debug/FallbackX", debugLeftFallbackX);
        SmartDashboard.putNumber("Vision/Left/Debug/FallbackY", debugLeftFallbackY);
        SmartDashboard.putNumber("Vision/Left/Debug/FallbackRot", debugLeftFallbackRot);
    }

    // ==================
    // VISION CONTROL METHODS
    // ==================

    /**
     * Enable vision pose updates. When enabled, vision measurements will be sent to the drivetrain.
     */
    public void enableVision() {
        visionEnabled = true;
    }

    /**
     * Disable vision pose updates. When disabled, vision will still detect targets but won't
     * update the robot's pose estimate. Useful for testing or when vision data is unreliable.
     */
    public void disableVision() {
        visionEnabled = false;
    }

    /**
     * Toggle vision enabled state.
     */
    public void toggleVision() {
        visionEnabled = !visionEnabled;
    }

    /**
     * Check if vision pose updates are currently enabled.
     * @return true if vision is enabled, false otherwise
     */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }
}
