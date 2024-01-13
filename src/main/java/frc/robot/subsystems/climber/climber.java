// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class climber extends SubsystemBase {
  /** Creates a new climber. */

      CANSparkMax tubeMotorLeft = new CANSparkMax(Constants.Climber.tubeMotor_Left_ID, MotorType.kBrushless);
    int state = 0;
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
  public void move_tube(boolean direction) {
    tubeMotorLeft.set(1);

  }

  // This tightens and loosens the winch
  public void move_winch() {

  }

  public void Perform_Step1() {    // Step 1 - Extend the tube in tube and winch
    if(state == 1) {
      state = 1;
      //if ((tubeMotorLeft.getdistance() > 4) && (winchMotorLeft.getdistance() > 4.0)){
        //move_tube(true);
        //move_winch(true);
      //}
      //else {
        // else we are done
//        state = 2;
      //}/
    }

  }
   

  // Step 2 - Move the robot forward


  public void Perform_Step3() {     // Step 3 - Retract the tube in tube AND winch
    state = 3;
    // if Tube and Winch distance is greater than 0.5 inches
    // then move the tube and winch
    // else we are done     state = 4;
  }


}