// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class climber extends SubsystemBase {
  /** Creates a new climber. */

    //state variables! 


      //CANSparkMax tubeMotorLeft = new CANSparkMax(Constants.Climber.tubeMotor_Left_ID, MotorType.kBrushless);
      //CANSparkMax tubeMotorRight = new CANSparkMax(Constants.Climber.tubeMotor_Right_ID, MotorType.kBrushless);

      TalonFX tubeMotorLeft = new TalonFX(Constants.Climber.tubeMotor_Left_ID);
      TalonFX tubeMotorRight = new TalonFX(Constants.Climber.tubeMotor_Right_ID);


      int leftState = 0;
      int rightState = 0; 
      XboxController controller = new XboxController(0);
      //1 - down; -1 - up
      int direction = 0;
      /* TODO:
        - setDirection() changes the variable depending on direction
        - figure joysticks
        - don't need Transit state?!
       */

      //  0 = starting state
      //  1 = Moving to prepare to climb
      //  2 = in climbing position
      //  3 = attempting to climb
      //  4 = completed climbing
      //  5 = error state

  public climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // This moves the tube in tube up and down
  public void move_tube() {
    //check the leftside of the tube
  if (controller.getXButton()) {
    climbUp();
  }

  else if (controller.getYButton()) {
    climbDown();
  }

  else {
    climbStop();
  }
}

  public void climbUp() {
    tubeMotorLeft.set(0.3);
    tubeMotorRight.set(0.3);
    SmartDashboard.putString("power input for robot", "0.3");
}

  public void climbDown() {
    tubeMotorLeft.set(-0.3);
    tubeMotorRight.set(-0.3);
    SmartDashboard.putString("power input for robot", "-0.3");
}

  public void climbStop() {
    tubeMotorLeft.set(0);
    tubeMotorRight.set(0);
    SmartDashboard.putString("power input for robot", "0"); 
  }
}