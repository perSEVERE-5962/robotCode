package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

/**
 * Move to Position
 */
public class StopDrive extends Command {

    public SwerveSubsystem swerve;


    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     */
    public StopDrive(SwerveSubsystem swerve) {
        this.swerve=swerve;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}