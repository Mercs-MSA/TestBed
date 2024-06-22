// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.VecBuilder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Pigeon2 pigeon2 = new Pigeon2(16);


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    LimelightHelpers.SetRobotOrientation("limelight", pigeon2.getAngle(), 0, 0, 0, 0, 0);

    // var lastResult = LimelightHelpers.getLatestResults("limelight");
    // if (lastResult.valid) {
    //   m_robotContainer.drivetrain.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight"), Timer.getFPGATimestamp());
    //   SmartDashboard.putBoolean("limelightResultValid", true);
    // } else {
    //   SmartDashboard.putBoolean("limelightResultValid", false);
    // }

    boolean doRejectUpdate = false;

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

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
      
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targets_Fiducials;

    boolean canSeeTag7 = false;
    for (int i = 0; i < fiducials.length; i++) {
      if (fiducials[i].fiducialID == 7) {
        canSeeTag7 = true;
        break;
      }
    }
    
    SmartDashboard.putBoolean("tag 7", canSeeTag7);
    SmartDashboard.putString("odometry", m_robotContainer.drivetrain.getOdometry().toString());
    SmartDashboard.putNumber("poseX", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("poseY", m_robotContainer.drivetrain.getState().Pose.getY());
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
