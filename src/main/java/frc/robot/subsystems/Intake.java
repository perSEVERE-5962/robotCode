// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.UltrasonicAnalog;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private UltrasonicAnalog intakeUltrasonic;
  /** Creates a new Intake. */
  public Intake(boolean isinverted,int motorId, UltrasonicAnalog intakeUltrasonic) {
    this.intakeUltrasonic = intakeUltrasonic;
    intakeMotor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(isinverted);

  }


  public void run(double speed) {
    intakeMotor.set(speed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public UltrasonicAnalog geUltrasonicAnalog(){
    return intakeUltrasonic;
  }
}
