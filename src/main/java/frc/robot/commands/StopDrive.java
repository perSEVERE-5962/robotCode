package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
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