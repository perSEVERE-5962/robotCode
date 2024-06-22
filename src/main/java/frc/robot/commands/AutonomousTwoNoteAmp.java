package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AutonomousTwoNoteAmp extends SequentialCommandGroup{
    public AutonomousTwoNoteAmp(){
        addCommands(
            ConditionalCommand(
                NewMoveToPosition(SwerveSubsystem.getInstance(), new Pose2d())
            )
        )
    }
}
