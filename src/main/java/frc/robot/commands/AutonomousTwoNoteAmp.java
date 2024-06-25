package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class AutonomousTwoNoteAmp extends SequentialCommandGroup{
    public AutonomousTwoNoteAmp(){
        addCommands(
            new ConditionalCommand(
                //blue team
                new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(-0.331, -0.432,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.3, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(0.0, 0.65, 0).withTimeout(0.25)).andThen(new ParallelCommandGroup(
                        new IntakeNote(),
                        new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(1.433, 0.258,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.3, DriveConstants.KPID_TKP).withTimeout(3))).alongWith(new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(-0.331, -0.432,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.3, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(0.0, 0.65, 0).withTimeout(3))),
                

                //red team
                new MoveToPosition(SwerveSubsystem.getInstance(),
                 new Pose2d(0.331, -0.432,
                    new Rotation2d(Units.degreesToRadians(0))),
                    0.3, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(0.0, 0.65, 0).withTimeout(0.25)),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            )
        );
            //From right up against the amp we have to go to go 47.64 inches to the left and 39.9 inches forward, while running the intake
    }
}
