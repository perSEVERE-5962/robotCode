/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.sensors.PIDControl;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import java.io.File;
import java.io.IOException;

public class PathFollow extends CommandBase {
  /**
   * 
   * Creates a new PathFollow.
   */
  public int intialPosition = 0;
  public int ticksPerRotation = 4096;
  public double wheelDiameter = 0.5;
  public EncoderFollower leftFollower;
  public EncoderFollower rightFollower;
  private Drive drive;
  private PIDControl configTalon;
  private AHRS gyro;
  final double kP = 0.3; //0.25
  final double kI = 0;
  final double kD = 0; //0.001
  final double kV = 1/7.83;//1/22.72; (ACTUAL VELOCITY: 12.3 ACTUAL ACCELERATION: 10)
  final double kA = 1/6;//1/72.29;

  public double leftPos;
  public double rightPos;

  public double leftCalculate;
  public double rightCalculate;

  public PathFollow(Drive drive, PIDControl configTalon, AHRS gyro)
   {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive); 
    // configTalon.configTalons();
    try{
    //need to implement PIDControl, Encoders??
    leftFollower = follower(Pathfinder.readFromCSV(new File("/home/lvuser/deploy/Straight13Feet_left.csv")));
    //add left location
    rightFollower = follower(Pathfinder.readFromCSV(new File("/home/lvuser/deploy/Straight13Feet_right.csv")));   
    }catch(IOException ex){
      System.err.println(ex);
    }
    this.drive = drive;
    this.configTalon = configTalon;
    this.gyro = gyro;
    

    configTalon.configTalons(drive);
    
  }
  public EncoderFollower follower(Trajectory trajec) {
    EncoderFollower encFollower = new EncoderFollower(trajec);
    encFollower.configureEncoder(intialPosition, ticksPerRotation, wheelDiameter);
    encFollower.configurePIDVA(kP, kI, kD, kV, kA);
    return encFollower;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftPos = drive.leftTalon().getSelectedSensorPosition();
    rightPos = drive.rightTalon().getSelectedSensorPosition();
    leftCalculate = leftFollower.calculate((int) leftPos);
    rightCalculate = rightFollower.calculate((int) rightPos);

    double gyro_heading = gyro.getAngle();   // Assuming the gyro is giving a value in degrees
    double desired_heading = leftFollower.getHeading();  // Should also be in degrees

    // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    angleDifference = angleDifference % 360.0;
    if (Math.abs(angleDifference) > 180.0) {
      angleDifference  = (angleDifference > 0) ? angleDifference - 360 : angleDifference  + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * angleDifference;

    drive.tankDrive(-1*(leftCalculate - turn), -1*(rightCalculate + turn));
    // System.out.println("gyro" + gyro_heading);
    // System.out.println("desired" + desired_heading);
    // System.out.println("angleDiff" + angleDifference);
    // System.out.println(leftCalculate + "/" + turn + "/" + rightCalculate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.err.println("Stopping Pathplanner!");
    drive.endPathFollower();
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftFollower.isFinished() && rightFollower.isFinished();

  }
}
