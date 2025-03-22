package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToCoralStationWithTags;
public class MoveAndScoreCoral extends SequentialCommandGroup {
    public MoveAndScoreCoral(int scorePosition, boolean isRightPost){

        addCommands(
            /* new SetArmPosition(scorePosition).andThen */(new MoveToCoralStationWithTags(isRightPost))
        );
    }
}