// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid m_dsol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid m_dsol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private Compressor m_pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    // m_pcmCompressor.enableDigital();
  }

  public void close() {
    m_dsol1.set(DoubleSolenoid.Value.kForward);
    m_dsol2.set(DoubleSolenoid.Value.kForward);
  }

  public void open() {
    m_dsol1.set(DoubleSolenoid.Value.kReverse);
    m_dsol2.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
