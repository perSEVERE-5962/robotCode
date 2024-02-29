// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.sensors.UltrasonicAnalog;

public class Intake extends SubsystemBase {
  private static Intake instance;

  private CANSparkMax intakeMotor;
  private UltrasonicAnalog intakeUltrasonic;
   //private Solenoid intakeSolenoid;

  /** Creates a new Intake. */
  private Intake() {
    intakeMotor = new CANSparkMax(Constants.CANDeviceIDs.kIntakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(Constants.MiscSubsystemConstants.kIntakeInverted);
    // intakeSolenoid = new Solenoid(Constants.CANDeviceIDs.kPCMID24V,
    //                               PneumaticsModuleType.CTREPCM,
    //                               Constants.UltrasonicConstants.kIntake_PCM_Channel);
    // intakeSolenoid.set(true);
    // intakeUltrasonic = new UltrasonicAnalog(UltrasonicConstants.kIntake_Analog_Channel);
  }

  public void run(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Intake Ultrasonic", getUltrasonicAnalog().getRange());
  }

  // public UltrasonicAnalog getUltrasonicAnalog() {
  //   if (intakeUltrasonic == null) {
  //     intakeUltrasonic = new UltrasonicAnalog(UltrasonicConstants.kIntake_Analog_Channel);
  //   }
  //   return intakeUltrasonic;
  // }

  // public boolean isInRange() {
  //   double range = intakeUltrasonic.getRange();
  //   SmartDashboard.putNumber("InTake Ultrasonic", range);
  //   if (range <= 11) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }
}
