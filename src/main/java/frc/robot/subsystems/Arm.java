
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm instance;
  private CANSparkMax m_ArmSpark;

  private RelativeEncoder m_encoder;

  public Arm() {
    m_ArmSpark =
        new CANSparkMax(
            Constants.CANDeviceIDs.kArmID,
            CANSparkLowLevel.MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
     * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
     * persist between power cycles
     */
    //    m_ArmSpark.restoreFactoryDefaults();

    m_ArmSpark.setInverted(false);
    m_encoder = m_ArmSpark.getEncoder();
    m_encoder.setPosition(0);
    m_ArmSpark.getPIDController().setP(Constants.ArmPIDCoeffients.kP);
    m_ArmSpark.getPIDController().setI(Constants.ArmPIDCoeffients.kI);
    m_ArmSpark.getPIDController().setD(Constants.ArmPIDCoeffients.kD);
    m_ArmSpark.getPIDController().setIZone(Constants.ArmPIDCoeffients.kIz);
    m_ArmSpark.getPIDController().setFF(Constants.ArmPIDCoeffients.kFF);
    m_ArmSpark
        .getPIDController()
        .setOutputRange(
            Constants.ArmPIDCoeffients.kMinOutput, Constants.ArmPIDCoeffients.kMaxOutput);

  }

  public void moveArm(double speed) {
    m_ArmSpark.set(speed);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveToPositionWithPID(double position) {
    m_ArmSpark.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

    /**
   * @return the instance
   */
  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }
}
