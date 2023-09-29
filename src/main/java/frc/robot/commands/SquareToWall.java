// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.lang.Math;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class SquareToWall extends CommandBase {
  protected SwerveSubsystem m_driveTrain;
  
  private final float KP = 0.1f;
  private final float ALIGNMENT_THRESHOLD = 2;


  
  private NetworkTable table;

  private float angleSign;


  private static double squish(double x)
  {
      return x / (1 + Math.abs(x));
  }


  private double TryGetAngle()
  {
    table = NetworkTableInstance.getDefault().getTable("laser_scan");
    double angle = 0;
    try
    {
      angle = Math.toDegrees(table.getValue("angle_to_move").getDouble());
    }
    catch (ClassCastException ex)
    {
      SmartDashboard.putString("Error", ex.getMessage());
    }
    return angle;
  }

  private float GetSign()
  {
    double angle = TryGetAngle();
    SmartDashboard.putString("Angle to turn: ", Double.toString(angle));
    

    float sign = 1.0f;

    

    if (angle > 0.0)
    {
      sign = -1.0f;
      SmartDashboard.putString("Angle is greater than 0: ", Double.toString(angle));
    }

    SmartDashboard.putString("Angle sign: ", Float.toString(sign));
    return sign;
  }


  /** Creates a new Forward. */
  public SquareToWall(SwerveSubsystem driveTrain) {

    this.m_driveTrain = driveTrain;

    table = NetworkTableInstance.getDefault().getTable("laser_scan");

    

    SmartDashboard.putString("Angle to turn: ", Double.toString(TryGetAngle()));

    
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize()
  {
    angleSign = GetSign();
  }


  /*
   * Add all this to the initialization:
   * 
   * turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
      turnController.setInputRange(-180.0f,  180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(kToleranceDegrees);
      turnController.setContinuous(true);
      
      /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
      /* tuning of the Turn Controller's P, I and D coefficients.            */
      /* Typically, only the P value needs to be modified.                   */
      //LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
  


  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
 
    float error = (float)TryGetAngle();

    chassisSpeeds = new ChassisSpeeds(0, 0, squish( -error * KP) / 2);
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    m_driveTrain.setModuleStates(moduleStates);

  }

    
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    double angle = TryGetAngle();
    SmartDashboard.putString("Angle to turn: ", Double.toString(angle));

    if (Math.abs(angle) < ALIGNMENT_THRESHOLD)
    {
      return true;
    }
    return false;
  }
}
