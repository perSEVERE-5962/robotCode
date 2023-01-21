// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid dsol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid dsol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  public void close() {
    dsol1.set(DoubleSolenoid.Value.kForward);
    dsol2.set(DoubleSolenoid.Value.kForward);
  }

  public void open() {
    dsol1.set(DoubleSolenoid.Value.kReverse);
    dsol2.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
