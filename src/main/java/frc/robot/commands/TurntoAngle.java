// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
 * This command will turn the robot to a specified angle.
 */
public class TurntoAngle extends Command {

    private SwerveSubsystem swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController =
        new HolonomicDriveController(
            new PIDController(DriveConstants.kPID_XKP, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
            new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
            new ProfiledPIDController(DriveConstants.KPID_TKP,DriveConstants.KPID_TKI, DriveConstants.KPID_TKD,
                DriveConstants.kThetaControllerConstraints));
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private int finishCounter = 0;

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */
    public TurntoAngle(SwerveSubsystem swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;
        holonomicDriveController.setTolerance(new Pose2d(2, 1.0, Rotation2d.fromDegrees(0.75)));
    }

    @Override
    public void initialize() {
        startPos = swerve.getPose();
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     targetPose2d = new Pose2d(targetPose2d.getTranslation(),
        //         targetPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        // }
        //SmartDashboard.putString("Target pos", targetPose2d.toString());
    }

    @Override
    public void execute() {
        //SmartDashboard.putString("Turn Status", "In progress");
        //SmartDashboard.putString("Current pos", swerve.getPose().toString());
        Pose2d currPose2d = swerve.getPose();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupt) {
        swerve.stopModules();
        //SmartDashboard.putString("Turn Status", "Done");
    }

    @Override
    public boolean isFinished() {

        if (holonomicDriveController.atReference() || (swerve.isAtAngle(goal))) {
            finishCounter++;
        } else {
            finishCounter = 0;
        }
        //SmartDashboard.putNumber("TurntoAngle finishCounter", finishCounter);
        return finishCounter > 2;
    }
}