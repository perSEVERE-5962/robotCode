// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPos4 extends SequentialCommandGroup {

  /** Creates a new Movebackshoot. */
  public AutoPos4(Intake intake, DriveTrain driveTrain, Arm arm, AHRS gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new LowerArm(arm),
        new ParallelCommandGroup(
            new AutoRunIntake(-0.75, intake), new AutoDriveForward(-49, driveTrain)),
        new AutoRunIntake(0, intake),
        new GyroRightTurn(driveTrain, gyro, 170),
        new RaiseArm(arm),
        new AutoDriveForward(-69, driveTrain),
        new MoveToShootPosition(arm),
        new AutoRunIntake(1, intake));
  }
}
