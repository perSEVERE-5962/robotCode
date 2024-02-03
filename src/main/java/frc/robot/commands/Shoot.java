// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooterFeeder;
import frc.robot.commands.SpinUpShooter;
import frc.robot.RobotContainer;
import frc.robot.Constants.ColorConstants;
import frc.robot.commands.ChangeLED;
import frc.robot.commands.StopShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RobotContainer instance = RobotContainer.getInstance();
    addCommands(new SpinUpShooter(instance.getShooter()),
        new RunShooterFeeder(instance.getFeeder(), instance.getFeederSensor()),
        new ChangeLED(instance.getNotification(), ColorConstants.YellowHue),
        new StopShooter(instance.getShooter()));
  }
}
