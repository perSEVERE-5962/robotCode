// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.obsolete.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private static Pneumatics instance;

  /** Creates a new Pneumatics. */
  private Pneumatics() {}

  public DoubleSolenoid add_double_solenoid(int channel1, int channel2) {
    DoubleSolenoid new_solenoid =
        new DoubleSolenoid(
            Constants.CANDeviceIDs.kPCMID, PneumaticsModuleType.CTREPCM, channel1, channel2);
    return new_solenoid;
  }

  public void forward(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void backward(DoubleSolenoid solenoid) {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * @return the instance
   */
  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }

    return instance;
  }
}
