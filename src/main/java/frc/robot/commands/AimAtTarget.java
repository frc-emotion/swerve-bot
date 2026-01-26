package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraSource;
import frc.robot.subsystems.Vision.TargetWithSource;

/**
 * Command to aim the robot at a visible AprilTag target.
 * Uses the target's yaw from the camera to rotate the robot.
 * 
 * Based on PhotonVision documentation:
 * https://docs.photonvision.org/en/v2026.0.1-beta/docs/examples/aimingatatarget.html
 */
public class AimAtTarget extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final int targetTagId; // -1 means any target
    
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // PID controller for smooth rotation (same gains as D-pad heading controller)
    private final PIDController rotationController = new PIDController(
        VisionConstants.VISION_TURN_kP,
        0,
        VisionConstants.VISION_TURN_kD
    );

    // Slew rate limiters to prevent current spikes when velocity changes
    // The rotation limiter smooths the PID output to prevent aggressive rotation starts
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(8.0); // rad/s²

    private double lastTargetYaw = 0.0;
    private boolean hasSeenTarget = false;
    
    /**
     * Creates an AimAtTarget command that aims at any visible AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param forwardSupplier Supplier for forward velocity (allows driver input while aiming)
     * @param strafeSupplier Supplier for strafe velocity (allows driver input while aiming)
     */
    public AimAtTarget(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier) {
        this(drivetrain, vision, forwardSupplier, strafeSupplier, -1);
    }
    
    /**
     * Creates an AimAtTarget command that aims at a specific AprilTag.
     * 
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param forwardSupplier Supplier for forward velocity
     * @param strafeSupplier Supplier for strafe velocity
     * @param targetTagId The specific AprilTag ID to aim at (-1 for any)
     */
    public AimAtTarget(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier,
            int targetTagId) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.targetTagId = targetTagId;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        hasSeenTarget = false;
        lastTargetYaw = 0.0;
        rotationController.reset();
        rotationController.setSetpoint(0); // Target is 0 yaw (centered)
    }
    
    @Override
    public void execute() {
        // Get driver inputs
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();
        double rotationRate = 0.0;

        // Find the target with camera source info
        Optional<TargetWithSource> targetWithSourceOpt;
        if (targetTagId < 0) {
            targetWithSourceOpt = vision.getBestTargetWithSource();
        } else {
            // For specific tag ID, we need to check which camera has it
            targetWithSourceOpt = vision.getBestTargetWithSource()
                .filter(tws -> tws.target().getFiducialId() == targetTagId);
        }

        // ENHANCED DEBUG OUTPUT
        SmartDashboard.putBoolean("AimAtTarget/ACTIVE", true);
        SmartDashboard.putBoolean("AimAtTarget/HasTarget", targetWithSourceOpt.isPresent());
        SmartDashboard.putNumber("AimAtTarget/ForwardInput", forward);
        SmartDashboard.putNumber("AimAtTarget/StrafeInput", strafe);

        if (targetWithSourceOpt.isPresent()) {
            TargetWithSource targetWithSource = targetWithSourceOpt.get();
            PhotonTrackedTarget target = targetWithSource.target();
            CameraSource cameraSource = targetWithSource.source();

            double targetYawDegrees = target.getYaw();
            lastTargetYaw = targetYawDegrees;
            hasSeenTarget = true;

            // Convert yaw to radians
            double targetYawRadians = Math.toRadians(targetYawDegrees);

            // Calculate rotation rate using PID control (outputs rad/s directly)
            // We want yaw to be 0 (centered in that camera's view), so we calculate output from current yaw
            // Negative because positive yaw means target is to the left of camera center,
            // so we need to rotate counter-clockwise (positive rotation)
            rotationRate = -rotationController.calculate(targetYawRadians);

            // Clamp to max angular rate
            rotationRate = MathUtil.clamp(rotationRate, -getMaxAngularRate(), getMaxAngularRate());

            // Apply slew rate limiting to prevent current spikes from sudden rotation changes
            rotationRate = rotationLimiter.calculate(rotationRate);

            // Enhanced debug output
            SmartDashboard.putNumber("AimAtTarget/TagID", target.getFiducialId());
            SmartDashboard.putString("AimAtTarget/CameraSource", cameraSource.toString());
            SmartDashboard.putNumber("AimAtTarget/TargetYawDeg", targetYawDegrees);
            SmartDashboard.putNumber("AimAtTarget/TargetYawRad", targetYawRadians);
            SmartDashboard.putNumber("AimAtTarget/TargetPitch", target.getPitch());
            SmartDashboard.putNumber("AimAtTarget/TargetArea", target.getArea());
            SmartDashboard.putNumber("AimAtTarget/PoseAmbiguity", target.getPoseAmbiguity());
            SmartDashboard.putNumber("AimAtTarget/RotationRate_RadPerSec", rotationRate);
            SmartDashboard.putNumber("AimAtTarget/kP", VisionConstants.VISION_TURN_kP);
            SmartDashboard.putNumber("AimAtTarget/kD", VisionConstants.VISION_TURN_kD);
            SmartDashboard.putBoolean("AimAtTarget/OnTarget", Math.abs(targetYawDegrees) < VisionConstants.AIM_TOLERANCE_DEGREES);
        } else {
            // No target visible
            SmartDashboard.putNumber("AimAtTarget/TagID", -1);
            SmartDashboard.putString("AimAtTarget/CameraSource", "NONE");
            SmartDashboard.putNumber("AimAtTarget/TargetYawDeg", 0.0);
            SmartDashboard.putNumber("AimAtTarget/TargetYawRad", 0.0);
            SmartDashboard.putNumber("AimAtTarget/TargetPitch", 0.0);
            SmartDashboard.putNumber("AimAtTarget/TargetArea", 0.0);
            SmartDashboard.putNumber("AimAtTarget/PoseAmbiguity", 0.0);
            SmartDashboard.putNumber("AimAtTarget/RotationRate_RadPerSec", 0.0);
            SmartDashboard.putBoolean("AimAtTarget/OnTarget", false);
        }

        // Apply the drive request (rotationRate is already in rad/s)
        drivetrain.setControl(
            driveRequest
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withRotationalRate(rotationRate)
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        // Clear active flag
        SmartDashboard.putBoolean("AimAtTarget/ACTIVE", false);

        // Stop the drivetrain rotation but allow momentum to carry
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
    
    @Override
    public boolean isFinished() {
        // This command runs until interrupted (button released)
        // Override this behavior by wrapping with .until() if needed
        return false;
    }
    
    /**
     * Returns whether the robot is currently aimed at the target.
     */
    public boolean isOnTarget() {
        return hasSeenTarget && Math.abs(lastTargetYaw) < VisionConstants.AIM_TOLERANCE_DEGREES;
    }
    
    /**
     * Gets the maximum angular rate from TunerConstants.
     * Using a reasonable default if not accessible.
     */
    private double getMaxAngularRate() {
        // ~1.5 rotations per second = ~9.42 rad/s
        return Math.PI * 3;
    }
}
