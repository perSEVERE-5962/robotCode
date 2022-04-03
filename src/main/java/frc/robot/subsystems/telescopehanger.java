// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class telescopehanger extends SubsystemBase {
  private WPI_TalonSRX m_telescopeControl;
  
  public telescopehanger() {
    m_telescopeControl = new WPI_TalonSRX(Constants.MotorControllerDeviceID.telescopingDeviceID);
    m_telescopeControl.setInverted(true); 
    
  }

  public void telescopeControl(double speed) {
    m_telescopeControl.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
