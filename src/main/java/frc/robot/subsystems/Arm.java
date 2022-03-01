// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax m_ArmSpark;

  private RelativeEncoder m_encoder;

  public Arm() {
    m_ArmSpark =
        new CANSparkMax(
            Constants.MotorControllerDeviceID.armDeviceID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
     * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
     * persist between power cycles
     */
    m_ArmSpark.restoreFactoryDefaults();

    m_ArmSpark.setInverted(false);

    m_encoder = m_ArmSpark.getEncoder();
    m_encoder.setPosition(0);

    /**
     * Soft Limits restrict the motion of the motor in a particular direction at a particular point.
     * Soft limits can be applied in only one direction, or both directions at the same time.
     *
     * <p>If the soft limits are disabled and then re-enabled, they will retain the last limits that
     * they had for that particular direction.
     *
     * <p>The directions are rev::CANSparkMax::kForward and rev::CANSparkMax::kReverse
     */
    m_ArmSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_ArmSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_ArmSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
    m_ArmSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -23.9f);
  }

  public void moveArm(double speed) {
    m_ArmSpark.set(speed);
    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    // -20 is lowest
  }
  public double getPosition(){
    return m_encoder.getPosition(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
