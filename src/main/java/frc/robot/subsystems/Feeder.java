// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.Constants.MiscSubsystemConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.sensors.RockwellUltrasonic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private static Feeder instance;

  private CANSparkMax feederMotor;
  private RockwellUltrasonic feederUltrasonic;
  private RockwellUltrasonic feederUltrasonic2;
  private Solenoid feederSolenoid;
  private Solenoid feeder2Solenoid;

  /** Creates a new Intake. */
  private Feeder() {
    feederMotor = new CANSparkMax(CANDeviceIDs.kFeederMotorID, CANSparkLowLevel.MotorType.kBrushless);
    feederMotor.setInverted(MiscSubsystemConstants.kFeederInverted);
    feederSolenoid = new Solenoid(
        CANDeviceIDs.kPCMID24V,
        PneumaticsModuleType.CTREPCM,
        UltrasonicConstants.kFeeder_PCM_Channel);
    feeder2Solenoid = new Solenoid(
        CANDeviceIDs.kPCMID24V,
        PneumaticsModuleType.CTREPCM,
        UltrasonicConstants.kFeeder2_PCM_Channel);
    feederSolenoid.set(true);
    feeder2Solenoid.set(true);

    feederUltrasonic = new RockwellUltrasonic(UltrasonicConstants.kFeeder_Analog_Channel); 
    feederUltrasonic2 = new RockwellUltrasonic(UltrasonicConstants.kFeeder2_Analog_Channel); 
  }
  public RockwellUltrasonic getUltrasonicOne(){
    return feederUltrasonic;
  }
  public RockwellUltrasonic getUltrasonicTwo(){
    return feederUltrasonic2;
  }

  public void run(double speed) {
    feederMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public boolean isInRange() {
    double range_of_feeder = feederUltrasonic.getRange();
    double range_of_2feeder = feederUltrasonic2.getRange();
    // SmartDashboard.putNumber("Feeder Ultrasonic 1", range_of_feeder);
    // SmartDashboard.putNumber("Feeder Ultrasonic 2", range_of_2feeder);
    if (range_of_feeder <= Constants.UltrasonicConstants.kMaxRange || 
        range_of_2feeder <= Constants.UltrasonicConstants.kMaxRange) {
      return true;
    } else {
      return false;
    }
  }

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    }
    return instance;
  }
}