/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.sensors.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathFollowAndIntake extends ParallelCommandGroup {
  /**
   * Creates a new PathFollowAndIntake.
   */
  public static Drive drive = new Drive();
  private static PIDControl configTalon = new PIDControl();

  public PathFollowAndIntake() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new PathFollow(drive, configTalon, drive.getGyro()), new RunIntake());
  }
}
