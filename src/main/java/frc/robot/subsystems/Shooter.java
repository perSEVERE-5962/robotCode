// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;

  /** Creates a new Intake. */
  private Shooter() {
    // Top shooter
    topMotor = new CANSparkMax(Constants.CANDeviceIDs.kShooter1MotorID, MotorType.kBrushless);
    topShooterEncoder = topMotor.getEncoder();
    topMotor.setIdleMode(IdleMode.kCoast);
    
    // Bottom shooter
    bottomMotor = new CANSparkMax(Constants.CANDeviceIDs.kShooter2MotorID, MotorType.kBrushless);
    bottomShooterEncoder = bottomMotor.getEncoder();
    bottomMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setSmartCurrentLimit(30);
    bottomMotor.setSmartCurrentLimit(30);
  }

  public void runShooter(double speed) {
      topMotor.set(-speed);
      bottomMotor.set(speed);
  }

  public void runTopShooter(double speed) {
    topMotor.set(-speed);
  }

  public void runBottomShooter(double speed) {
    bottomMotor.set(speed);
  }

  public double getTopVelocity() {
    double velocity = topShooterEncoder.getVelocity();
    return velocity;
  }

  public double getBottomVelocity() {
    double velocity = bottomShooterEncoder.getVelocity();
    return velocity;
  }

  public double getAverageVelocity() {
    double topVelocity = getTopVelocity();
    double bottomVelocity = getBottomVelocity();
    double averageVelocity = (topVelocity + bottomVelocity) / 2.0;
    return averageVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}

