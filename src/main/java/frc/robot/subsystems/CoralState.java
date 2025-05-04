// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.moveSubsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class CoralState extends SubsystemBase {
  private static CoralState instance;
  private boolean isdetected = true;
  private DigitalInput leftIR = new DigitalInput(4);
  private CoralState() {

  }

  @Override
  public void periodic() {
      isdetected  = leftIR.get();
      SmartDashboard.putBoolean("Coral detected", isdetected);
  }
  public boolean isdetected(){
    return isdetected;
  }
  public static CoralState getInstance(){
    if (instance == null) {
      instance = new CoralState();
    }

    return instance;
  }
  }

