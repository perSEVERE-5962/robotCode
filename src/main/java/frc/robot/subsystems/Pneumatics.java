// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private Compressor m_pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    m_pcmCompressor.enableDigital();
  }

  public DoubleSolenoid add_double_solenoid(int channel1, int channel2) {
    DoubleSolenoid new_solenoid =
        new DoubleSolenoid(PneumaticsModuleType.CTREPCM, channel1, channel2);
    return new_solenoid;
  }

  public void forward(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void backward(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}