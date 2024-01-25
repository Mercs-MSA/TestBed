// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//  USE NEXT LINE FOR TESTING
//  import frc.robot.sim.PhysicsSim;

/* Work In Progress
 * DONE Install library for IR brake sensor
 * ON HOLD Use sensors to connect intake and SAT
 * Find out what motors we're using and how to program them
 * ON HOLD Refactor when rollers begin spinning
 * ON HOLD use/connect robotState to our code
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
  int intakeArmState;
  int intakeRollerState;
  boolean intakeSensorState;
  final int stateArmUp = 1;
  final int stateArmDown = 2;
  final int stateArmMoving = 3;
  final int stateRollerNotMoving = 4;
  final int stateRollerMoving = 5;
  final double speedRollerInward = 1.0;
  final double speedRollerOutward = -1.0;
  final double positionArmDown = 0.001;
  final double positionArmUp = 2.1;

  DutyCycleEncoder intakeRollerMotor = new DutyCycleEncoder(IntakeConstants.INTAKE_ROLLER_MOTOR_ID);
  DigitalInput intakeSensor = new DigitalInput(0);
  XboxController controller = new XboxController(5);
  private final TalonFX m_fx = new TalonFX(1, "rio");
  private final PositionVoltage m_voltagePosition = new PositionVoltage(positionArmUp, 0, true, 0, 0, false, false, false);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new intake. */
  public intake() {
    // This method will be called once (at the beginning)

    // declare that the starting state of intake is armUp and Roller is not moving
    intakeArmState = stateArmUp;
    intakeRollerState = stateRollerNotMoving;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // USE NEXT LINE FOR TESTING
    // PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  // USE FOR TESTING ALSO
  // @Override
  // public void simulationPeriodic() {
  //   PhysicsSim.getInstance().run();
  // }

  @Override
  public void periodic() {
    m_fx.getPosition().refresh();

    // This method will be called once per scheduler run
    if (m_fx.getPosition().getValue() >= positionArmUp) {
      intakeArmState = stateArmUp;
      m_fx.setControl(m_brake);        
    }
    else if (m_fx.getPosition().getValue() <= positionArmDown) {
      intakeArmState = stateArmDown;
      m_fx.setControl(m_brake);
    }

    // constantly check sensor for note
    if (intakeSensor.get()) {
      // this is the state where it doesn't detect the orage (for some reason)
      intakeSensorState = false;
    }
    else {
      // when the note is present it changes states
      intakeSensorState = true;
    }

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
    // TODO: When arm starts moving, run noteIn

    SmartDashboard.putBoolean("IsButtonPressed", controller.getAButton());
    SmartDashboard.putNumber("StateDetection", intakeArmState);
    SmartDashboard.putBoolean("Detecting Note", intakeSensorState);
    SmartDashboard.putNumber("MotorPosition", m_fx.getPosition().getValue());
  }

  // Release intake system opening
  public void intakeDown() {
    // if the arm is up,
    if (intakeArmState == stateArmUp) {
      // you should move the arm down
      m_fx.setControl(m_voltagePosition.withPosition(positionArmDown));
      intakeArmState = stateArmMoving;
    }
  }
  
  // Picking the intake system opening back up
  public void intakeUp() {
    // If the arm is down,
    if (intakeArmState == stateArmDown) {
      // you should move it up
      m_fx.setControl(m_voltagePosition.withPosition(positionArmUp));
      intakeArmState = stateArmMoving;
    }
  }

  // Activate the intake wheels
  public void noteIn() {
    intakeRollerState = stateRollerMoving;
    // intakeRollerMotor.set(speedRollerInward);
  }

  // Optional mode that can release a note
  public void noteOut() {
    intakeRollerState = stateRollerMoving;
    // intakeRollerMotor.set(speedRollerOutward);
  }

  public void stopRoller() {
    //set rollor speed to 0
    intakeRollerState = stateRollerNotMoving;
    // intakeRollerMotor.set(0);
  }
}