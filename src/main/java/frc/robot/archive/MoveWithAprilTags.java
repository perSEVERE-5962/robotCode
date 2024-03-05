// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveWithAprilTags extends Command {
  private SwerveSubsystem driveTrain;
  private double speedX = 0.0;
  private boolean validX = false;
  private double speedZ = 0.0;
  private boolean validZ = false;
  private double speedRot = 0.0;
  private boolean validRot = false;

  /** Creates a new Forward. */
    public MoveWithAprilTags(SwerveSubsystem m_driveTrain) {
    this.driveTrain = m_driveTrain;
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.       read in the xyz
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    double xPos = SpeakerTagInfo.tag1Info.getPos().getX();
    double zPos = SpeakerTagInfo.tag1Info.getPos().getZ();
    double yRot = SpeakerTagInfo.tag1Info.getRot().getY();
    if (xPos > 0.5) { // Too far right
      speedX = -0.5;
      validX = false;
    } else if (xPos < -0.5) { // Too far left
      speedX = 0.5;
      validX = false;
    } else { // In position
      validX = true;
    }

    if (zPos > 1) { // Too far backwards
      speedZ = 0.5;
      validZ = false;
    } else if (zPos < 1) { // Too far forwards
      speedZ = -0.5;
      validZ = false;
    } else { // In position
      validZ = true;
    }

    if (yRot > 0.5) {
      speedRot = -0.5;
      validRot = false;
    } else if (yRot < -0.5) {
      speedRot = 0.5;
      validRot = false;
    } else { //in position
      validRot = true;
    }

    chassisSpeeds = new ChassisSpeeds(speedX, speedZ, speedRot);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  // Returns true when the command should end.    check to see if all are zero pat will givr values
  @Override
  public boolean isFinished() {
    return validX && validZ && validRot;
  }
}
