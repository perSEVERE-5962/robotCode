// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vec3;
import frc.robot.subsystems.DetectAprilTags;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AlignToSpeaker extends Command {
  /** Creates a new AlignToSpeaker. */
  SwerveSubsystem m_driveTrain = SwerveSubsystem.getInstance();
  public AlignToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find both april tags for more precise calculations
    int aptag1Index = DetectAprilTags.getAprilTagIndexFromId(Constants.SpeakerConstants.kSpeakerAprilTag1Id);
    int aptag2Index = DetectAprilTags.getAprilTagIndexFromId(Constants.SpeakerConstants.kSpeakerAprilTag2Id);
    if (aptag1Index != -1 && aptag2Index != -1) {
      // Get relative position to calculated origin
      Vec3 aptag1Pos = DetectAprilTags.getAprilTagPos(aptag1Index);
      Vec3 aptag2Pos = DetectAprilTags.getAprilTagPos(aptag2Index);
      Vec3 robotPos = new Vec3((aptag1Pos.x - aptag2Pos.x)/2, 0 /*y pos is unnecessary */, (aptag1Pos.z - aptag2Pos.z)/2);
      Vec3 posDiff = robotPos.difference(Constants.SpeakerConstants.kOriginFromSpeaker);

      // Need to move to a specific location using only position values
      // Figure out how to translate position values into how fast each wheel should move

      // Always face the april tag
      double angleToTag = Math.toDegrees(Math.atan2(aptag1Pos.x, aptag1Pos.y));
      // Turn so that the robot will face towards the april tag
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
