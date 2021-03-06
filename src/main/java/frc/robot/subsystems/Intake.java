/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static WPI_VictorSPX motorControl;

  /**
   * Creates a new Intake.
   */
  public Intake(){
    motorControl = new WPI_VictorSPX(19);
    motorControl.setInverted(true);
}
  
public void runIntake(){
  motorControl.set(0.75);
}

public void shoot(){
  motorControl.set(-0.75);
}

public void stop(){
  motorControl.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
