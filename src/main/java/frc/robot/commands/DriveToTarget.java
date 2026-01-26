package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Command to drive the robot to a specified distance from an AprilTag target.
 * Uses pitch angle and trigonometry to calculate distance.
 * 
 * Based on PhotonVision documentation:
 * https://docs.photonvision.org/en/v2026.0.1-beta/docs/examples/aimandrange.html
 */
public class DriveToTarget extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final DoubleSupplier strafeSupplier;
    private final double targetDistanceMeters;
    private final int targetTagId; // -1 means any target
    
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private double currentDistance = 0.0;
    private boolean hasSeenTarget = false;
    
    /**
     * Creates a DriveToTarget command that drives to any visible AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param strafeSupplier Supplier for strafe velocity (allows driver input)
     * @param targetDistanceMeters Desired distance from the target in meters
     */
    public DriveToTarget(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            DoubleSupplier strafeSupplier,
            double targetDistanceMeters) {
        this(drivetrain, vision, strafeSupplier, targetDistanceMeters, -1);
    }
    
    /**
     * Creates a DriveToTarget command that drives to a specific AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param strafeSupplier Supplier for strafe velocity
     * @param targetDistanceMeters Desired distance from the target in meters
     * @param targetTagId The specific AprilTag ID to target (-1 for any)
     */
    public DriveToTarget(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            DoubleSupplier strafeSupplier,
            double targetDistanceMeters,
            int targetTagId) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.strafeSupplier = strafeSupplier;
        this.targetDistanceMeters = targetDistanceMeters;
        this.targetTagId = targetTagId;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        hasSeenTarget = false;
        currentDistance = 0.0;
    }
    
    @Override
    public void execute() {
        // Get driver strafe input
        double strafe = strafeSupplier.getAsDouble();
        double forward = 0.0;
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
            
            // Get tag height for distance calculation
            double tagHeight = VisionConstants.getTagHeight(target.getFiducialId());
            
            // Calculate distance using trigonometry
            // Distance = (tagHeight - cameraHeight) / tan(cameraPitch + targetPitch)
            currentDistance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAM_RIGHT_HEIGHT_METERS, // Use average camera height
                tagHeight,
                VisionConstants.CAM_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch())
            );
            
            // Calculate forward velocity to reach target distance
            double distanceError = currentDistance - targetDistanceMeters;
            forward = distanceError * VisionConstants.VISION_DRIVE_kP;
            forward = MathUtil.clamp(forward, -1.0, 1.0);
            
            // Also aim at the target while driving
            double targetYaw = target.getYaw();
            rotationRate = -targetYaw * VisionConstants.VISION_TURN_kP;
            rotationRate = MathUtil.clamp(rotationRate, -1.0, 1.0);
            
            SmartDashboard.putNumber("DriveToTarget/CurrentDistance", currentDistance);
            SmartDashboard.putNumber("DriveToTarget/DistanceError", distanceError);
            SmartDashboard.putNumber("DriveToTarget/ForwardSpeed", forward);
            SmartDashboard.putNumber("DriveToTarget/TagID", target.getFiducialId());
        }
        
        SmartDashboard.putBoolean("DriveToTarget/HasTarget", targetOpt.isPresent());
        SmartDashboard.putBoolean("DriveToTarget/AtDistance", isAtTargetDistance());
        
        // Apply the drive request
        drivetrain.setControl(
            driveRequest
                .withVelocityX(forward * getMaxLinearSpeed())
                .withVelocityY(strafe)
                .withRotationalRate(rotationRate * getMaxAngularRate())
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
        // Run until interrupted
        return false;
    }
    
    /**
     * Returns whether the robot is at the target distance.
     */
    public boolean isAtTargetDistance() {
        if (!hasSeenTarget) return false;
        return Math.abs(currentDistance - targetDistanceMeters) < VisionConstants.RANGE_TOLERANCE_METERS;
    }
    
    /**
     * Gets the current measured distance to the target.
     */
    public double getCurrentDistance() {
        return currentDistance;
    }
    
    private double getMaxLinearSpeed() {
        return 4.0; // ~4 m/s, reasonable for approaching targets
    }
    
    private double getMaxAngularRate() {
        return Math.PI * 3; // ~1.5 rotations per second
    }
}
