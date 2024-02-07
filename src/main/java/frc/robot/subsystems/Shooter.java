// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  private RelativeEncoder topShooterEncoder;
  private RelativeEncoder bottomShooterEncoder;
  /** Creates a new Intake. */
  public Shooter(int kShooter1MotorID, int kShooter2MotorID) {
    topMotor = new CANSparkMax(kShooter1MotorID, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(kShooter2MotorID, MotorType.kBrushless);
    topShooterEncoder = topMotor.getEncoder();
    bottomShooterEncoder = bottomMotor.getEncoder();
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
  }

public void runShooter(double speed) {
    topMotor.set(-1*speed);
    bottomMotor.set(speed);
    
}
public double getVelocity(){
  double velocity = (topShooterEncoder.getVelocity() + bottomShooterEncoder.getVelocity())/2;
  SmartDashboard.putNumber("Shooter average velocity: ", getVelocity());
  return velocity;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

