// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.obsolete;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Move;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetOnChargingStation extends ParallelRaceGroup {
  /** Creates a new GetOnChargingStation. */
  public GetOnChargingStation(SwerveSubsystem driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Move(driveTrain, -1, 0, 0), new CheckIfEngaged(driveTrain));
  }
}