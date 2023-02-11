// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LineDetector;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_LeaveCommunityAndEngage extends SequentialCommandGroup {
  /** Creates a new GroupSeqCom_MovePastLineAndBack. */
  public AUTO_LeaveCommunityAndEngage(Drivetrain drivetrain, LineDetector lineDetector, AHRS gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GroupParRace_MoveToLine(drivetrain, lineDetector),
      new ForwardDistance(drivetrain, -1, 10 /* Unknown so far */), // Moves foward fully past the line
      new GroupSeqCom_EngageChargingStation(drivetrain, gyro)
    );
  }
}