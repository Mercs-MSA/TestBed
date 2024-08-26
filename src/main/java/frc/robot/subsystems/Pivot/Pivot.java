package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Pivot extends SubsystemBase{
    // Hardware
    private final TalonFX leaderTalon;

    // Status Signals
    private final StatusSignal<Double> internalPositionRotations;
    private final StatusSignal<Double> velocityRps;
    private final StatusSignal<Double> appliedVoltage;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> tempCelsius;
    private final PositionVoltage leaderVoltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // Config
    private final TalonFXConfiguration configLeader = new TalonFXConfiguration();

    //Check if inverted is applied 
    public static Boolean status1OK = false; 
    public static Boolean status2OK = false;

    private double targetPose;

    public Pivot(){
        leaderTalon = new TalonFX(ArmConstants.leaderID, "rio");

        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Leader motor configs
        configLeader.Slot0.kP = ArmConstants.leaderKP;
        configLeader.Slot0.kI = ArmConstants.leaderKI;
        configLeader.Slot0.kD = ArmConstants.leaderKD;
        configLeader.Voltage.PeakForwardVoltage = 16;
        configLeader.Voltage.PeakReverseVoltage  = -16;
        configLeader.MotorOutput.Inverted =
        ArmConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeader.Feedback.RotorToSensorRatio = ArmConstants.rotorToSensorRatio;
        configLeader.Feedback.SensorToMechanismRatio = ArmConstants.sensorToMechanismRatio;
        leaderTalon.getConfigurator().apply(configLeader, 1.0);

        // Status signals
        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        appliedVoltage = leaderTalon.getMotorVoltage();
        supplyCurrent = leaderTalon.getSupplyCurrent();
        tempCelsius = leaderTalon.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        tempCelsius);

        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 20; ++i) {
            status1 = leaderTalon.getConfigurator().apply(configLeader);
            if (status1.isOK())
                status1OK = true;
                SmartDashboard.putBoolean("Status 1", Pivot.status1OK);
                break;
        }
        if (!status1.isOK()) {
            System.out.println("Could not apply configs, error code: " + status1.toString());
        }

        leaderTalon.setPosition(0);        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Leader Motor Temperature", leaderTalon.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Leader Motor Position", leaderTalon.getPosition().getValueAsDouble());
    }

    public void setBrakeMode(boolean enabled){
        leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public double getLeaderPos(){
        return leaderTalon.getPosition().getValueAsDouble();
    }

    public void leaderGoToPositionIncrement(double increment) {
        targetPose = targetPose + (increment*2);
        leaderTalon.setControl(leaderVoltagePosition.withPosition(targetPose));
    }

    public void leaderGoToPosition(double pos) {
        leaderTalon.setControl(leaderVoltagePosition.withPosition(pos));
    }

    public void stop(){
        leaderTalon.setControl(new NeutralOut());
    }

    public void performAutoHome(){
        // something something...
        leaderTalon.setPosition(0);
    }

    public void applyConfig(){
        leaderTalon.getConfigurator().apply(configLeader);
    }
}
