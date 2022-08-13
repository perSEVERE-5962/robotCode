/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new AutoSquence. */
  public Autonomous() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        // In this example the robot will drive forward,
        // turn to the left and then to the right,
        // and then drive backward.  Is should end up
        // in the starting position (or close to it)
        new Forward(10), // move forward 10 inches
        new TurnLeft(90), // turn to the left 90 degrees
        new TurnRight(90), // turn to the right 90 degrees
        new Backward(10), // move backward 10 inches
        new Stop() // stop the robot from moving
        );
  }
}
