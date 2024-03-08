// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ! Just use SwerveSubsystem !

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wheels extends SubsystemBase {
  private CANSparkMax m_leadMotor;
  // private CANSparkMax m_followMotor;
  CANcoder m_encoder;

  /** Creates a new Wheels. */
  public Wheels(int motorId, int encoderId) {
    m_leadMotor =
        new CANSparkMax(motorId, MotorType.kBrushless);
    // m_followMotor = new CANSparkMax(
    // Constants.CANDeviceIDs.kReachFollowID,
    // com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    m_leadMotor.setInverted(true);

    m_encoder = new CANcoder(encoderId, Constants.DriveConstants.kCanBusName);
    
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    m_encoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    m_encoder.getPosition().setUpdateFrequency(100);
    m_encoder.getVelocity().setUpdateFrequency(100);    
  }

  public void turn(double speed) {
    m_leadMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
