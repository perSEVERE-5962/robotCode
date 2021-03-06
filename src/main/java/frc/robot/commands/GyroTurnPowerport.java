/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;;
public class GyroTurnPowerport extends CommandBase {
  /**
   * Creates a new gyroTurn180.
   */
  // public RobotContainer robotC = new RobotContainer();
  private Drive subsystem;
  private AHRS ahrs;

  public GyroTurnPowerport(Drive subsystem, AHRS ahrs) {
    addRequirements(subsystem);

    this.subsystem = subsystem;
    this.ahrs = ahrs;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ahrs.reset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      subsystem.driveRight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished Turning!");
    subsystem.stopDrive();
    // subsystem.resetEncoders();
    ahrs.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ahrs.getAngle() > 15;
  }
}