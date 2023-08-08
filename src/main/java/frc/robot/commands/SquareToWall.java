// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.lang.Math;

public class SquareToWall extends Move {
  private final float ALIGNMENT_THRESHOLD = 1;

  
  private NetworkTable table;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public SquareToWall(SwerveSubsystem driveTrain) {

    super(driveTrain, 0, 0, 1);

    
    table = NetworkTableInstance.getDefault().getTable("laser_scan");

    double angle = Math.toDegrees(table.getValue("angle_to_move").getDouble());

    SmartDashboard.putString("Angle to turn: ", Double.toString(angle));

    
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public boolean isFinished() {
    double angle = Math.toDegrees(table.getValue("angle_to_move").getDouble());

    SmartDashboard.putString("Angle to turn: ", Double.toString(angle));

    if (Math.abs(angle) < ALIGNMENT_THRESHOLD) {
      return true;
    }
    return false;
  }
}
