// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;

  /** Creates a new Intake. */
  public Feeder(boolean isinverted,int motorId) {
    feederMotor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
    feederMotor.setInverted(isinverted);

  }


  public void run(double speed) {
    feederMotor.set(speed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
