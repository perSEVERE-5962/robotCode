// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  private final boolean noteRequired=true ;
  
  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Feeder feeder, UltrasonicAnalog feederSensor, Notification changeLight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    boolean checkForNote=changeLight.getNoteState() ;
    if (checkForNote == noteRequired) {
      addCommands(new SpinUpShooter(shooter),
          new RunShooterFeeder(feeder, feederSensor),
          new ChangeLED(changeLight, false),
          new StopShooter(shooter));
    }
  }
}
