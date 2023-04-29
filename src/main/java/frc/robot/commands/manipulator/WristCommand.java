// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.manipulator.Wrist;

public class WristCommand extends CommandBase {
  XboxController controller = RobotContainer.getInstance().getCopilotController();
  Wrist wrist = Wrist.getInstance();

  /** Creates a new WristCommand. */
  public WristCommand() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Wrist.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getRawAxis(1) * -1;
    wrist.move(MathUtil.applyDeadband(speed, 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Wrist.getInstance().move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
