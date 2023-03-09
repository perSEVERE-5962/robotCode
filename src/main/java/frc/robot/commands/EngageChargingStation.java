// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EngageChargingStation extends SequentialCommandGroup {

  /** Creates a new EngageChargingStation. */
  public EngageChargingStation(SwerveSubsystem driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*
     * Face perpendicular to the charging station //Assume already true
     * Drive forwards onto the charging station
     * Stop once the charging station becomes level
     */
    addCommands(
        new GetOnChargingStation(driveTrain), // Drive until pitch changes
        new StayEngaged(driveTrain));
    // new StopDrive(driveTrain));
  }
}
