/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.sensors.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CameraLight;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoSequence extends SequentialCommandGroup {
  /**
   * Creates a new AutoSquence.
   */
  private RobotContainer robotContainer;
  // public static Drive drive = new Drive();
  // public static Intake intake = new Intake();
  //public static Arm arm = new Arm();
  // private static PIDControl configTalon = new PIDControl();
  public AutoSequence(Arm arm, CameraLight light, Drive drive, Intake intake, PIDControl pidControl) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new MoveArmToIntake(arm), 
    // new WaitCommand(0.5), 
    new PathFollowAndIntake(drive, pidControl, intake),
    new StopIntake(intake),
    new ResetDriveEncoders(drive),
    new PathFollowStraight(drive, pidControl, drive.getGyro()),
    new StopDrive(drive),
     new GyroTurn(drive, drive.getGyro()),
     new StopDrive(drive),
     new MoveArmVision(arm),

    // new WaitCommand(0.5),
    new TurnOnLight(light),
    //new WaitCommand(0.1),
    new FindTarget(drive),
    new StopDrive(drive),
    new MoveArmToShoot(arm, 13),
     new Squaretopowerport(drive),
    new StopDrive(drive),
    new ShootShakeRight(drive, arm, intake)
    );
  }
}
