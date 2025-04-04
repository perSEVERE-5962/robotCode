// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoToReefWithCamera extends SequentialCommandGroup {
  /** Creates a new AutoToTroughWithCamera. */
  public AutoToReefWithCamera() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     final double target=Constants.StartingPos.poseTransform2d.getX()/*+Constants.StartingPos.poseTransform2d_2.getX())/2*/-0.3;

    addCommands(//new GetAprilTagDistance(),
                 new MoveToPosition(SwerveSubsystem.getInstance(),
                         new Pose2d(Constants.StartingPos.poseTransform2d.getX()/*+Constants.StartingPos.poseTransform2d_2.getX())/2*/-0.3, 0.5, new Rotation2d(0)),
                         0.5, DriveConstants.KPID_TKP).withTimeout(10),
              //  new SetArmPosition(Constants.ScoringConstants.kAutoL1),
                new ScoreCoral(-0.1).withTimeout(0.1),
                new ScoreCoral(-0.1).withTimeout(0.1),
                new ScoreCoral(-0.1).withTimeout(2)
                
    );
    
  }
}
