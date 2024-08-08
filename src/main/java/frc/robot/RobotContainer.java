// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("NothingAuto");

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> 
      drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(limelight_aim_proportional())
    ));
    joystick.y().onTrue(
      new InstantCommand(() -> {
        SmartDashboard.putNumber("Pose X: ", limelightAlignToAmp().getX());
        SmartDashboard.putNumber("Pose Y: ", limelightAlignToAmp().getY());
      })
      // limelightAlignToAmp(), 
      // new PathConstraints(
      //   4.0, 4.0, 
      //   Units.degreesToRadians(360), Units.degreesToRadians(540)
      // ), 
      // 0, 
      // 0
    );
    joystick.x().onTrue(AutoBuilder.pathfindToPose(
      new Pose2d(1.80, 7.6, Rotation2d.fromDegrees(90)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    ));


    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

// TESTING WOOHOOOO

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      0
    ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  




  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }

  public double limelight_aim_proportional() {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targets_Fiducials;

    double targetingAngularVelocity = 0;

    for (int i = 0; i < fiducials.length; i++) {
      if (fiducials[i].fiducialID == 6) {
        double kP = 0.035;
        targetingAngularVelocity = fiducials[i].tx * kP;
        targetingAngularVelocity *= MaxAngularRate;
        targetingAngularVelocity *= -1.0;
        break;
      }
    }
    
    return targetingAngularVelocity;
  }



  public Pose2d limelightAlignToAmp() {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targets_Fiducials;

    
    SmartDashboard.putString("HasCodeExecuted1", "1");
    Pose2d targetPosition = new Pose2d();
    SmartDashboard.putNumber("Something", fiducials.length);
    for (int i = 0; i < fiducials.length; i++) {
      SmartDashboard.putNumber("Fish", fiducials[i].fiducialID);
      if (fiducials[i].fiducialID == 6) {
        SmartDashboard.putString("HasCodeExecuted2", "2");
        targetPosition = fiducials[i].getRobotPose_TargetSpace2D();
        
        break;
      }
    }
    
    return targetPosition;
  }

}






