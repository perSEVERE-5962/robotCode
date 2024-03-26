// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWithApriltag extends SequentialCommandGroup {
  Shooter shooter = Shooter.getInstance();
  Feeder feeder = Feeder.getInstance();
  Notification notification = Notification.getInstance();
  /** Creates a new ShootWithApriltag. */
  public ShootWithApriltag() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TurnToApriltag().withTimeout(2),
      new MoveToShootDistance()/*,
      new SpinUpShooter(1, 1, 0).withTimeout(1),
      new RunShooterFeeder(feeder,notification),
      new StopShooter(shooter)*/
    );
  }
}
