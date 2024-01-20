// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.XboxController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

/* Work In Progress
 * Install library for REV 2m distance sensor
 * Use sensors to connect intake and SAT
 * Find out what motors we're using and how to program them
 * Refactor when rollers begin spinning
 * use/connect robotState to our code
 */

/* Q&A:
 * 1 Have you figured out what motors you want to use?
 *   Using vortexes for motors for now
 * 
 * 2 How does the mechanism work?
 *   System drops to floor from bottom of robot and takes in notes
 *   Request for buttons to control 
 *   Holding button down keeps the system down and releasing button moves system back up
 * 
 * 3 Have you decided if there will be special sensors?
 *   Using the REV 2m distance senor
 * 
 * 4 How does the note get to the SAT system?
 *   An index that uses rollers to transport it into the SAT, not part of the intake
 * 
 * 5 How does the machine realize it has an onion ring?
 *   Sensors inside the index, but it's not important to the intake system itself.
 * 
 * 6 What button do you want to be assigned to operate the intake system?
 *   TODO:
 */

public class intake extends SubsystemBase {

  CANSparkMax intakeArmMotor = new CANSparkMax(IntakeConstants.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax intakeRollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
// add definition for the sensor here
  XboxController controller = new XboxController(0);

  int intakeArmState;
  int intakeRollerState;
  // add boolean for sensor state
  final int stateArmUp = 1;
  final int stateArmDown = 2;
  final int stateArmMoving = 3;
  final int stateRollerNotMoving = 4;
  final int stateRollerMoving = 5;
  final double speedRollerInward = 1.0;
  final double speedRollerOutward = -1.0;
  final double speedArmUp = 1.0;
  final double speedArmDown = -1.0;

  /** Creates a new intake. */
  public intake() {
    // This method will be called once (at the beginning)
  
    // declare that the starting state of intake is armUp and Roller is not moving
    intakeArmState = stateArmUp;
    intakeRollerState = stateRollerNotMoving;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getAngle() == 45) {
      intakeArmState = stateArmDown;        
    }
    else if (getAngle() == 0) {
      intakeArmState = stateArmUp;
    }

    // constantly check sensor for distance
    // when the distance changes it means to note is present and it changes states

    // if button A is held,
    if (controller.getAButton()) {
      // start intakeDown
      intakeDown();

    }

    // if button A is released,
    else {
      // start intakeUp
      intakeUp();

    }

    // if button B is pressed and the rollers are not moving,
    if (controller.getBButton() && intakeRollerState == stateRollerNotMoving) {
      // start rolling out
      noteOut();
    }

    // if button B is released and the rollers are moving,
    else if (controller.getBButton() == false && intakeRollerState == stateRollerMoving) {
      // stop rolling
      stopRoller();
    }

    // TODO: get 2 sensors to set boundries for the arm to reach
    
    // TODO: When arm starts moving, run noteIn
  }

  // Release intake system opening
  public void intakeDown() {
    // if the arm is up,
    if (intakeArmState == stateArmUp) {
      // you should move the arm down
      setAngle(45);
      intakeArmState = stateArmMoving;
    }
    // If the arm is down,
    //else if (intakeArmState == stateArmDown) {
      // you shouldn't move it down
      
    //}
    // If the arm is moving,
    //else {
      // you should keep moving
      
    //}
  }
  
  // Picking the intake system opening back up
  public void intakeUp() {
    // If the arm is down,
    if (intakeArmState == stateArmDown) {
      // you should move it up
      setAngle(0);
      intakeArmState = stateArmMoving;


      // Stop roller motor
    }
    // if the arm is up,
    //else if (intakeArmState == stateArmUp) {
      // you shouldn't move the arm up
      
    //}
    // If the arm is moving,
   // else {
      // you should keep moving
      
    //}
  }

  // Activate the intake wheels
  public void noteIn() {
    intakeRollerState = stateRollerMoving;
    intakeRollerMotor.set(speedRollerInward);
  }

  // Optional mode that can release a note
  public void noteOut() {
    intakeRollerState = stateRollerMoving;
    intakeRollerMotor.set(speedRollerOutward);
  }

  public void stopRoller() {
    //set rollor speed to 0
    intakeRollerState = stateRollerNotMoving;
    intakeRollerMotor.set(0);
  }







  // THIS IS A PSEUDOCODE FUNCTION AND IS NOT REAL, EVENTUALLY THIS WILL BE IN SWERVEMODULE.JAVA
  public void setAngle(int angle)
  {
    // PSEUDOCODE, DOES NOTHING
  }

  public int getAngle() {
    return 1;
  }
}