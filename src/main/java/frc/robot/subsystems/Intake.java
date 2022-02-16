// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static WPI_VictorSPX motorControl;

  /** Creates a new Intake. */
  public Intake() {
    motorControl = new WPI_VictorSPX(19); // 19 is temporary
    motorControl.setInverted(true);
  }

  public void armIntake(double speed) {
    motorControl.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
