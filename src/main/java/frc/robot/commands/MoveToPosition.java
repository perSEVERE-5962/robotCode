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
public class MoveToPosition extends Command {

    public SwerveSubsystem swerve;
    public Pose2d pose2d;

    private HolonomicDriveController holonomicDriveController =
    new HolonomicDriveController(
        new PIDController(DriveConstants.kPID_XKP + 0.5, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
        new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
        new ProfiledPIDController(DriveConstants.KPID_TKP,DriveConstants.KPID_TKI, DriveConstants.KPID_TKD,
            DriveConstants.kThetaControllerConstraints));

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2d Pose2d
     */
    public MoveToPosition(SwerveSubsystem swerve, Pose2d pose2d, double tol) {
        this(swerve, pose2d, 0.05, DriveConstants.KPID_TKP);
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2d Pose2d
     */
    public MoveToPosition(SwerveSubsystem swerve, Pose2d pose2d, double tol, double turnP) {
        this.swerve = swerve;
        this.pose2d = pose2d;
        this.addRequirements(swerve);
        holonomicDriveController.getThetaController().setP(turnP);
        holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(1)));
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2d Pose2d
     */
    public MoveToPosition(SwerveSubsystem swerve, Pose2d pose2d) {
        this(swerve, pose2d, 0.05);
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     */
    public MoveToPosition(SwerveSubsystem swerve) {
        this(swerve, new Pose2d());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds =
            holonomicDriveController.calculate(swerve.getPose(), pose2d, 0, pose2d.getRotation());
        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}