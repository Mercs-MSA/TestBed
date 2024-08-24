// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.VecBuilder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Pigeon2 pigeon2 = new Pigeon2(16, "canivore");
  private boolean enableLimeLight = true;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    addLimeLightResultsToOdometry();

    // SmartDashboard.putBoolean("tag 7", limelightCanSeeAprilTag(7));
    // SmartDashboard.putString("odometry", m_robotContainer.drivetrain.getOdometry().toString());
    // SmartDashboard.putNumber("poseX", m_robotContainer.drivetrain.getState().Pose.getX());
    // SmartDashboard.putNumber("poseY", m_robotContainer.drivetrain.getState().Pose.getY());
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

  public void addLimeLightResultsToOdometry() {
    if (!enableLimeLight) {
      return;
    }

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight-front", m_robotContainer.drivetrain.getOdometry().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

    if(Math.abs(pigeon2.getRate()) > 720) {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    
    if(!doRejectUpdate) {
      SmartDashboard.putBoolean("limelightResultValid", true);
      m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_robotContainer.drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    } else {
      SmartDashboard.putBoolean("limelightResultValid", false);
    }
  }

  public boolean limelightCanSeeAprilTag(int tagNumber) {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targets_Fiducials;

    for (int i = 0; i < fiducials.length; i++) {
      if (fiducials[i].fiducialID == tagNumber) {
        return true;
      }
    }

    return false;
  }
}
