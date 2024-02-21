// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.sensors.UltrasonicAnalog;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private static Feeder instance;

  private CANSparkMax feederMotor;
  private UltrasonicAnalog feederUltrasonic;
  private Solenoid feederSolenoid;

  /** Creates a new Intake. */
  private Feeder() {
    feederMotor = new CANSparkMax(Constants.CANDeviceIDs.kFeederMotorID, CANSparkLowLevel.MotorType.kBrushless);
    feederMotor.setInverted(Constants.MiscSubsystemConstants.kFeederInverted);
    feederSolenoid = new Solenoid(Constants.CANDeviceIDs.kPCMID24V,
                                  PneumaticsModuleType.CTREPCM,
                                  Constants.UltrasonicConstants.kFeeder_PCM_Channel);
    feederSolenoid.set(true);
  }

  public void run(double speed) {
    feederMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public UltrasonicAnalog getUltrasonicAnalog() {
    if (feederUltrasonic == null){
      feederUltrasonic = new UltrasonicAnalog(UltrasonicConstants.kFeeder_Analog_Channel, 
                                              UltrasonicConstants.kFeeder_PCM_Channel);
    }
    return feederUltrasonic;
  }

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    }
    return instance;
  }
}