package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to automatically align the robot to an AprilTag target.
 * Combines aiming (yaw) and ranging (distance) into a single command.
 * 
 * Based on PhotonVision documentation:
 * https://docs.photonvision.org/en/v2026.0.1-beta/docs/examples/aimandrange.html
 */
public class AutoAlign extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final double targetDistanceMeters;
    private final int targetTagId;
    private final boolean finishWhenAligned;
    
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Slew rate limiters to prevent current spikes when velocity changes
    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(5.0); // m/s²
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(8.0); // rad/s²

    private double currentDistance = 0.0;
    private double currentYaw = 0.0;
    private boolean hasSeenTarget = false;
    private int alignedFrameCount = 0;
    
    // Number of consecutive frames we need to be aligned before considering done
    private static final int ALIGNED_FRAME_THRESHOLD = 10;
    
    /**
     * Creates an AutoAlign command that aligns to any visible AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param targetDistanceMeters Desired distance from the target in meters
     */
    public AutoAlign(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            double targetDistanceMeters) {
        this(drivetrain, vision, targetDistanceMeters, -1, false);
    }
    
    /**
     * Creates an AutoAlign command with full configuration.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param targetDistanceMeters Desired distance from the target in meters
     * @param targetTagId Specific AprilTag ID to align to (-1 for any)
     * @param finishWhenAligned If true, command ends when aligned; otherwise runs until interrupted
     */
    public AutoAlign(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            double targetDistanceMeters,
            int targetTagId,
            boolean finishWhenAligned) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;
        this.targetTagId = targetTagId;
        this.finishWhenAligned = finishWhenAligned;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        hasSeenTarget = false;
        currentDistance = 0.0;
        currentYaw = 0.0;
        alignedFrameCount = 0;
    }
    
    @Override
    public void execute() {
        double forward = 0.0;
        double strafe = 0.0;
        double rotationRate = 0.0;
        
        // Find the target
        Optional<PhotonTrackedTarget> targetOpt;
        if (targetTagId < 0) {
            targetOpt = vision.getBestTarget();
        } else {
            targetOpt = vision.getTargetById(targetTagId);
        }
        
        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            hasSeenTarget = true;
            
            // Get current yaw
            currentYaw = target.getYaw();
            
            // Calculate distance
            double tagHeight = VisionConstants.getTagHeight(target.getFiducialId());
            currentDistance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAM_RIGHT_HEIGHT_METERS,
                tagHeight,
                VisionConstants.CAM_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch())
            );
            
            // Calculate rotation rate (aim)
            rotationRate = -currentYaw * VisionConstants.VISION_TURN_kP;
            rotationRate = MathUtil.clamp(rotationRate, -1.0, 1.0);
            
            // Calculate forward velocity (range)
            double distanceError = currentDistance - targetDistanceMeters;
            forward = distanceError * VisionConstants.VISION_DRIVE_kP;
            forward = MathUtil.clamp(forward, -1.0, 1.0);
            
            // Check if aligned
            boolean yawAligned = Math.abs(currentYaw) < VisionConstants.AIM_TOLERANCE_DEGREES;
            boolean distanceAligned = Math.abs(distanceError) < VisionConstants.RANGE_TOLERANCE_METERS;
            
            if (yawAligned && distanceAligned) {
                alignedFrameCount++;
            } else {
                alignedFrameCount = 0;
            }
            
            // Log telemetry
            SmartDashboard.putNumber("AutoAlign/Yaw", currentYaw);
            SmartDashboard.putNumber("AutoAlign/Distance", currentDistance);
            SmartDashboard.putNumber("AutoAlign/DistanceError", distanceError);
            SmartDashboard.putNumber("AutoAlign/TagID", target.getFiducialId());
            SmartDashboard.putBoolean("AutoAlign/YawAligned", yawAligned);
            SmartDashboard.putBoolean("AutoAlign/DistanceAligned", distanceAligned);
        } else {
            // No target - stop moving
            alignedFrameCount = 0;
        }
        
        SmartDashboard.putBoolean("AutoAlign/HasTarget", targetOpt.isPresent());
        SmartDashboard.putBoolean("AutoAlign/IsAligned", isAligned());

        // Apply slew rate limiting to prevent current spikes from sudden velocity changes
        double limitedForward = forwardLimiter.calculate(forward * getMaxLinearSpeed());
        double limitedRotation = rotationLimiter.calculate(rotationRate * getMaxAngularRate());

        // Apply the drive request
        drivetrain.setControl(
            driveRequest
                .withVelocityX(limitedForward)
                .withVelocityY(strafe)
                .withRotationalRate(limitedRotation)
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
    
    @Override
    public boolean isFinished() {
        if (finishWhenAligned) {
            return isAligned();
        }
        return false;
    }
    
    /**
     * Returns whether the robot is fully aligned (both yaw and distance).
     */
    public boolean isAligned() {
        return hasSeenTarget && alignedFrameCount >= ALIGNED_FRAME_THRESHOLD;
    }
    
    /**
     * Returns whether the robot is aimed at the target (yaw only).
     */
    public boolean isAimed() {
        return hasSeenTarget && Math.abs(currentYaw) < VisionConstants.AIM_TOLERANCE_DEGREES;
    }
    
    /**
     * Returns whether the robot is at the target distance.
     */
    public boolean isAtDistance() {
        return hasSeenTarget && 
            Math.abs(currentDistance - targetDistanceMeters) < VisionConstants.RANGE_TOLERANCE_METERS;
    }
    
    private double getMaxLinearSpeed() {
        return 4.0;
    }
    
    private double getMaxAngularRate() {
        return Math.PI * 3;
    }
}
