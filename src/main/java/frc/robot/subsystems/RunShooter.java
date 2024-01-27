// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RunShooter extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  /** Creates a new Intake. */
  public RunShooter() {
    topMotor = new CANSparkMax(Constants.CANDeviceIDs.kShooter1MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Constants.CANDeviceIDs.kShooter2MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  }


  public void runShoot(double speed) {
    topMotor.set(speed);
    bottomMotor.set(-1*speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

