// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveTarmac extends ParallelCommandGroup {
  /** Creates a new LeaveTarmac. */
  public LeaveTarmac(Intake intake, DriveTrain driveTrain, Arm arm) {
    addCommands(
        new LowerArm(arm),
        new ParallelCommandGroup(
            new AutoRunIntake(-0.75, intake), new AutoDriveForward(-49, driveTrain)),
        new AutoRunIntake(0, intake));
  }
}
