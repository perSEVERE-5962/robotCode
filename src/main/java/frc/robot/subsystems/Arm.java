package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class Arm extends SubsystemBase{
    private CANSparkMax armMotor;
    private static Arm instance;
    private static RelativeEncoder armEncoder;

    public Arm(){
        armMotor = new CANSparkMax(Constants.CANDeviceIDs.kArmID, CANSparkLowLevel.MotorType.kBrushless);
        armMotor.setInverted(false);

        armMotor.getPIDController().setP(Constants.ArmConstants.kP);
        armMotor.getPIDController().setI(Constants.ArmConstants.kI);
        armMotor.getPIDController().setD(Constants.ArmConstants.kD);
        armMotor.getPIDController().setIZone(Constants.ArmConstants.kIz);
        armMotor.getPIDController().setFF(Constants.ArmConstants.kFF);

        armMotor
        .getPIDController()
        .setOutputRange(Constants.ArmConstants.kMinOutput, Constants.ArmConstants.kMaxOutput);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        armMotor.setSoftLimit(
        SoftLimitDirection.kForward, Constants.ArmConstants.kUpperSoftLimit);
        armMotor.setSoftLimit(
        SoftLimitDirection.kReverse, Constants.ArmConstants.kLowerSoftLimit);
    }
    public double getPosition() {
        return armEncoder.getPosition();
    }
    public void moveToPositionWithPID(double position) {
        armMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
      }
    
    @Override
    public void periodic(){

    }
    public static Arm getInstance() {
        if (instance == null) {
          instance = new Arm();
        }
    
        return instance;
      }
}
