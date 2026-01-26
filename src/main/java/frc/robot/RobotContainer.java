// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AimAtTarget;
import frc.robot.commands.AutoAlign;
import frc.robot.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * CONTROLLER LAYOUT:
 * ==================
 * Left Stick:
 *   - X-axis (left/right): Strafe left/right
 *   - Y-axis (up/down): Drive forward/backward
 *
 * Right Stick:
 *   - X-axis (left/right): Rotate robot counterclockwise/clockwise
 *
 * Buttons:
 *   - A Button: Apply brake (holds wheels in current position)
 *   - B Button: Point all wheels in direction of left stick
 *   - X Button: X Formation (defensive lock - resists pushing from all directions)
 *   - Y Button: Aim at AprilTag (auto-rotate to face target while allowing translation)
 *   - Left Bumper: Reset field-centric heading (zero gyro to current direction)
 *   - Right Bumper: Auto-align (aim + drive to 1m from AprilTag)
 *   - D-Pad Up: Lock rotation to 0° (forward) while allowing free translation
 *   - D-Pad Right: Lock rotation to 90° (right) while allowing free translation
 *   - D-Pad Down: Lock rotation to 180° (backward) while allowing free translation
 *   - D-Pad Left: Lock rotation to 270° (left) while allowing free translation
 *   - Back + A: Toggle vision pose updates (disable when testing with bad AprilTag data)
 *   - Back + B: Reset pose estimator to origin (clears accumulated odometry + vision)
 *   - Back + X: SysId Dynamic Reverse
 *   - Back + Y: SysId Dynamic Forward
 *   - Start + X: SysId Quasistatic Reverse
 *   - Start + Y: SysId Quasistatic Forward
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // ==================
    // SLEW RATE LIMITERS
    // ==================
    // These limit the rate of change of velocity commands to prevent current spikes
    // and voltage drops when accelerating from rest. Units are meters/second/second
    // for translation and radians/second/second for rotation.
    //
    // Lower values = slower acceleration = less current draw = more stable voltage
    // Higher values = faster acceleration = more responsive but higher current spikes
    //
    // Tune these based on your battery health and desired responsiveness:
    // - Start conservative (lower values like 3-4 m/s²) and increase if too sluggish
    // - If you still see voltage drops, decrease these values
    // - Typical competition values range from 3-8 m/s² depending on robot mass and battery

    /** Max translational acceleration in m/s² (X and Y directions) */
    private static final double kTranslationSlewRate = 5.0; // m/s² - tune this value

    /** Max rotational acceleration in rad/s² */
    private static final double kRotationSlewRate = 8.0; // rad/s² - tune this value

    // Separate limiters for each axis - each maintains its own state
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(kRotationSlewRate);

    /* Setting up bindings for necessary control of the swerve drive platform */

    /** Field-centric drive request for normal driving */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /** Brake request - holds wheels in current position to stop robot */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /** Point wheels request - aims all wheels in the direction of the left joystick */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /** X Formation request - locks wheels in X pattern for maximum resistance to pushing */
    private final SwerveRequest.PointWheelsAt xFormation = new SwerveRequest.PointWheelsAt();

    /** Field-centric drive with locked rotation angle - allows translation while holding heading */
    private final SwerveRequest.FieldCentricFacingAngle driveWithLockedRotation = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withRotationalDeadband(MaxAngularRate * 0.1); // Add rotational deadband

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Telemetry logger = new Telemetry(MaxSpeed, drivetrain);
    public final Vision vision = new Vision();

    public RobotContainer() {
        // Configure the heading controller for locked rotation driving
        // These gains control how aggressively the robot maintains its heading
        driveWithLockedRotation.HeadingController.setPID(8, 0, 0);
        driveWithLockedRotation.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        configureBindings();
        configureNetworkTablesButtons();
    }

    /**
     * Configure button bindings for the driver controller.
     *
     * Default Command: Field-centric driving using left stick for translation,
     *                  right stick X-axis for rotation.
     */
    private void configureBindings() {
        // ==================
        // DEFAULT COMMAND
        // ==================
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Slew rate limiters are applied to prevent current spikes and voltage drops
        // when accelerating from rest.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                // Apply slew rate limiting to joystick inputs before sending to drivetrain
                // This limits acceleration, spreading out current draw over time
                double xVelocity = xLimiter.calculate(-joystick.getLeftY() * MaxSpeed);
                double yVelocity = yLimiter.calculate(-joystick.getLeftX() * MaxSpeed);
                double rotRate = rotationLimiter.calculate(-joystick.getRightX() * MaxAngularRate);

                return drive.withVelocityX(xVelocity) // Drive forward with negative Y (forward)
                    .withVelocityY(yVelocity) // Drive left with negative X (left)
                    .withRotationalRate(rotRate); // Drive counterclockwise with negative X (left)
            })
        );

        // ==================
        // DISABLED MODE
        // ==================
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // ==================
        // A BUTTON - BRAKE
        // ==================
        // Applies brake to all drive motors, holding wheels in current orientation.
        // Actively resists movement but wheels stay in their current angle.
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // ==================
        // B BUTTON - POINT WHEELS
        // ==================
        // Points all 4 wheels in the direction of the left joystick.
        // Does not drive, just orients the wheels. Useful for setup or testing.
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // ==================
        // X BUTTON - X FORMATION (DEFENSIVE LOCK)
        // ==================
        // Forms an X pattern with the wheels for maximum resistance to pushing.
        // This is the best defensive position to prevent being moved by other robots.
        // Wheels are angled: FL=45°, FR=135°, BL=-45° (315°), BR=-135° (225°)
        joystick.x().whileTrue(drivetrain.applyRequest(() ->
            xFormation.withModuleDirection(new Rotation2d(Math.PI / 4)) // 45 degrees for X formation
        ));

        // ==================
        // D-PAD - LOCKED ROTATION DRIVING
        // ==================
        // D-pad locks the robot's rotation to cardinal directions while allowing free translation.
        // Robot will automatically maintain the locked heading while you drive around.

        // D-Pad Up: Lock rotation to 0° (facing forward)
        // Note: Slew rate limiters are applied to translation for consistent acceleration limiting
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                .withTargetDirection(Rotation2d.kZero) // 0 degrees
        ));

        // D-Pad Right: Lock rotation to 90° (facing right)
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                .withTargetDirection(Rotation2d.fromDegrees(90))
        ));

        // D-Pad Down: Lock rotation to 180° (facing backward)
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                .withTargetDirection(Rotation2d.k180deg) // 180 degrees
        ));

        // D-Pad Left: Lock rotation to 270° (facing left)
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                .withTargetDirection(Rotation2d.fromDegrees(270))
        ));

        // ==================
        // LEFT BUMPER - RESET GYRO
        // ==================
        // Resets the field-centric heading to make the current direction "forward".
        // Useful when starting in a specific orientation or if gyro drift occurs.
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // ==================
        // VISION COMMANDS
        // ==================
        // Y Button: Aim at the best visible AprilTag while allowing driver translation
        joystick.y().whileTrue(
            new AimAtTarget(
                drivetrain,
                vision,
                () -> -joystick.getLeftY() * MaxSpeed,  // Forward
                () -> -joystick.getLeftX() * MaxSpeed   // Strafe
            )
        );
        
        // Right Bumper: Auto-align to best visible AprilTag at 1 meter distance
        joystick.rightBumper().whileTrue(
            new AutoAlign(
                drivetrain,
                vision,
                VisionConstants.DEFAULT_TARGET_DISTANCE
            )
        );

        // ==================
        // SYSID ROUTINES (for characterization/tuning)
        // ==================
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ==================
        // VISION TESTING CONTROLS
        // ==================
        // Back + A: Toggle vision pose updates (useful when testing with AprilTags)
        joystick.back().and(joystick.a()).onTrue(Commands.runOnce(() -> {
            vision.toggleVision();
            System.out.println("Vision " + (vision.isVisionEnabled() ? "ENABLED" : "DISABLED"));
        }));

        // Back + B: Reset pose estimator (clears all accumulated odometry + vision data)
        joystick.back().and(joystick.b()).onTrue(Commands.runOnce(() -> {
            resetPoseAndGyro();
            System.out.println("Pose estimator reset to origin");
        }));

        // ==================
        // TELEMETRY
        // ==================
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configures NetworkTables buttons that appear in Elastic/SmartDashboard.
     * These provide clickable controls for vision testing and pose reset.
     */
    private void configureNetworkTablesButtons() {
        // Add button to reset pose estimator (appears in SmartDashboard/Elastic)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
            "Reset Pose to Origin",
            Commands.runOnce(() -> {
                resetPoseAndGyro();
                System.out.println("[Dashboard] Pose estimator reset to origin");
            }).ignoringDisable(true)
        );

        // Add button to toggle vision enabled/disabled
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
            "Toggle Vision",
            Commands.runOnce(() -> {
                vision.toggleVision();
                System.out.println("[Dashboard] Vision " + (vision.isVisionEnabled() ? "ENABLED" : "DISABLED"));
            }).ignoringDisable(true)
        );

        // Add button to enable vision
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
            "Enable Vision",
            Commands.runOnce(() -> {
                vision.enableVision();
                System.out.println("[Dashboard] Vision ENABLED");
            }).ignoringDisable(true)
        );

        // Add button to disable vision
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
            "Disable Vision",
            Commands.runOnce(() -> {
                vision.disableVision();
                System.out.println("[Dashboard] Vision DISABLED");
            }).ignoringDisable(true)
        );
    }

    /**
     * Resets the robot's pose estimator and zeroes the gyro heading.
     * This is a full reset - position to (0,0) and heading to 0°.
     * Clears all accumulated odometry and vision measurements.
     * Useful when testing vision or recovering from bad pose estimates.
     */
    public void resetPoseAndGyro() {
        drivetrain.seedFieldCentric();
    }

    private void configureAutonomousCommands() {
        autoChooser.setDefaultOption("None", Commands.none());
        // Add more auto commands here
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("1MeterTest");
    }
}
