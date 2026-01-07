// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
 *   - Y Button: [Available for future use]
 *   - Left Bumper: Reset field-centric heading (zero gyro to current direction)
 *   - Right Bumper: [Available for future use]
 *   - D-Pad Up: Lock rotation to 0° (forward) while allowing free translation
 *   - D-Pad Right: Lock rotation to 90° (right) while allowing free translation
 *   - D-Pad Down: Lock rotation to 180° (backward) while allowing free translation
 *   - D-Pad Left: Lock rotation to 270° (left) while allowing free translation
 *   - Back + X: SysId Dynamic Reverse
 *   - Back + Y: SysId Dynamic Forward
 *   - Start + X: SysId Quasistatic Reverse
 *   - Start + Y: SysId Quasistatic Forward
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
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
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
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
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.kZero) // 0 degrees
        ));

        // D-Pad Right: Lock rotation to 90° (facing right)
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(90))
        ));

        // D-Pad Down: Lock rotation to 180° (facing backward)
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.k180deg) // 180 degrees
        ));

        // D-Pad Left: Lock rotation to 270° (facing left)
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            driveWithLockedRotation
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(270))
        ));

        // ==================
        // LEFT BUMPER - RESET GYRO
        // ==================
        // Resets the field-centric heading to make the current direction "forward".
        // Useful when starting in a specific orientation or if gyro drift occurs.
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
        // TELEMETRY
        // ==================
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureAutonomousCommands() {
        autoChooser.setDefaultOption("None", Commands.none());
        // Add more auto commands here
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("1MeterTest");
    }
}
