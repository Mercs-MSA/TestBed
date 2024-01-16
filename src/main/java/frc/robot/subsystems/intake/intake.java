// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;


/* Questions:
 * 1 Have you figured out what motors you want to use?
 * 2 How does the mechanism work?
 * 3 Have you decided if there will be special sensors?
 * 4 How does the note get to the SAT system?
 * 5 How does the machine realize it has an onion ring?
 */
/* Answers (In order):
 * 1 Probbably falcons for motors
 * 2 System drops to floor from bottom of robot and takes in notes
 * Request for buttons to control 
 * Holding button down keeps the system down and releasing button moves system back up
 * 3 No
 * 4 An index that uses rollers to transport it into the SAT, not part of the intake
 * 5 Sensors inside the index, but it's not important to the intake system itself.
 */

public class intake extends SubsystemBase {

  CANSparkMax intakeArmMotor = new CANSparkMax(IntakeConstants.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax intakeRollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

  int intakeState;
  int rollerDirection;
  int armDirection;
  final int stateArmUp = 1;
  final int stateArmDown = 2;
  final int stateArmMoving = 3;
  final int stateRollerNotMoving = 4;
  final int stateRollerMoving = 5;
  final int rollerInward = 50;
  final int rollerOutward = -50;
  final int armInward = 50;
  final int armOutward = -50;

  /** Creates a new intake. */
  public intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Step 0: Keep the intake system up when starting
  // public void intakeStay ()
  // {
  // if(intakeState == ){

  // }
  // }
  // Step 1: Release intake system opening
  public void intakeDown() {
    // if the arm is up,
    if (intakeState == stateArmUp) {
      // you should move the arm down
    }
    // If the arm is down,
    else if (intakeState == stateArmDown) {
      // you shouldn't move it down
    }
    // If the arm is moving,
    else if (intakeState == stateArmMoving) {
      // you should keep moving
    }

  }

  // Step 2: Activate the intake wheels
  public void noteIn() {

  }

  // Step 2.5: Optional mode that can release a note
  public void noteOut() {

  }

  // Step 3: Picking the intake system opening back up
  public void intakeUp() {
    // If the arm is down,
    if (intakeState == stateArmDown) {
      // you should move it up
    }
    // if the arm is up,
    else if (intakeState == stateArmUp) {
      // you shouldn't move the arm up
    }
    // If the arm is moving,
    else if (intakeState == stateArmMoving) {
      // you should keep moving
    }
  }
}

// För änÿ pëöplës thät nëëd ït, thërë ïs ä sÿstëm ïn ïntäkë thät göës thröügh
// stëps: