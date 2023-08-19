// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.manipulator.Lift;
import frc.robot.subsystems.manipulator.Reach;
import frc.robot.subsystems.manipulator.Wrist;

public class MoveSubsystemsManual extends CommandBase {
  /** Creates a new MoveWristManual. */
  private Wrist m_wrist = Wrist.getInstance();
  private Reach m_reach = Reach.getInstance();
  private Lift m_lift = Lift.getInstance();

  private double voltage;

  public MoveSubsystemsManual(double voltage) {
    this.voltage = voltage;
    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController driverController = RobotContainer.getInstance().getDriverController();
    if (driverController.getAButton()) {
      m_wrist.moveWithVoltage(voltage); // Move wrist
    } else if (driverController.getBButton()) {
      m_reach.moveWithVoltage(voltage * 2); // Move reach
    } else if (driverController.getYButton()) {
      m_lift.moveWithVoltage(voltage / 3); // Move lift
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.moveWithVoltage(0);
    m_reach.moveWithVoltage(0);
    m_lift.moveWithVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
