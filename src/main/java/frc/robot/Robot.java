// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Update drivetrain pose estimator with vision measurements
    updateVisionMeasurements();
  }
  
  /**
   * Feeds vision pose estimates to the drivetrain's Kalman filter.
   * This fuses vision data with odometry for more accurate localization.
   */
  private void updateVisionMeasurements() {
    // Track poses for telemetry visualization
    edu.wpi.first.math.geometry.Pose2d rightPose = null;
    edu.wpi.first.math.geometry.Pose2d leftPose = null;
    
    // Process right camera estimate
    var rightEstimate = m_robotContainer.vision.getEstimatedPoseRight();
    if (rightEstimate.isPresent()) {
      var estimate = rightEstimate.get();
      rightPose = estimate.estimatedPose.toPose2d();
      
      m_robotContainer.drivetrain.addVisionMeasurement(
        rightPose,
        estimate.timestampSeconds,
        m_robotContainer.vision.getStdDevsRight()
      );
    }
    
    // Process left camera estimate
    var leftEstimate = m_robotContainer.vision.getEstimatedPoseLeft();
    if (leftEstimate.isPresent()) {
      var estimate = leftEstimate.get();
      leftPose = estimate.estimatedPose.toPose2d();
      
      m_robotContainer.drivetrain.addVisionMeasurement(
        leftPose,
        estimate.timestampSeconds,
        m_robotContainer.vision.getStdDevsLeft()
      );
    }
    
    // Update reference pose for vision estimators (helps with pose selection)
    m_robotContainer.vision.setReferencePose(
      m_robotContainer.drivetrain.getState().Pose
    );
    
    // Update telemetry with vision poses for Field2d visualization
    m_robotContainer.logger.updateVisionPoses(rightPose, leftPose);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
