/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.sensors.UltrasonicHCSR04;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drive extends SubsystemBase {
  private WPI_TalonSRX robotLeftTalon;
  private WPI_VictorSPX robotLeftVictor;
  private WPI_TalonSRX robotRightTalon;
  private WPI_VictorSPX robotRightVictor;
  private final double speedfactor = 1;
  private final double autospeedfactor = 0.275; // 0.23 for comp bot 
  // private double ultrasonicRange;
  // private boolean ultrasonicCheck = false;
  private double radius = 3;
  private double circumfrence = 2 * Math.PI * radius;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private NetworkTableEntry myEntry;
  private boolean pathFollowerDone = false;
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  public AHRS getGyro(){
    return ahrs;
  }
  public WPI_TalonSRX leftTalon() {
    return robotLeftTalon;
  }

  public WPI_TalonSRX rightTalon() {
    return robotRightTalon;
  }

  // public UltrasonicHCSR04 ultrasonic;
  // }

  public Drive() {
    robotRightTalon = new WPI_TalonSRX(23);
    robotRightVictor = new WPI_VictorSPX(20);
    robotLeftTalon = new WPI_TalonSRX(22);
    robotLeftVictor = new WPI_VictorSPX(21);
    robotLeftVictor.follow(robotLeftTalon, FollowerType.PercentOutput);
    robotRightVictor.follow(robotRightTalon, FollowerType.PercentOutput);
    robotLeftVictor.setInverted(false);
    robotLeftTalon.setInverted(false);
    robotRightVictor.setInverted(true);
    robotRightTalon.setInverted(true);
  }

  public WPI_TalonSRX getRobotRightTalon() {
    return robotRightTalon;
  }

  public WPI_TalonSRX getRobotLeftTalon() {
    return robotLeftTalon;
  }

  public WPI_VictorSPX getRobotRightVictor() {
    return robotRightVictor;
  }

  public WPI_VictorSPX getRobotLeftVictor() {
    return robotLeftVictor;
  }
  public void resetGyro() {
    ahrs.reset();
  }

  public double getGyroAngle() {
    return ahrs.getAngle();
  }
  private void setsmooth() {
    robotLeftTalon.configOpenloopRamp(0.7);
    robotLeftTalon.configClosedloopRamp(0);
    robotRightTalon.configOpenloopRamp(0.7);
    robotRightTalon.configOpenloopRamp(0);
  }

  private void unsetsmooth() {
    robotLeftTalon.configOpenloopRamp(0);
    robotLeftTalon.configClosedloopRamp(0);
    robotRightTalon.configOpenloopRamp(0);
    robotRightTalon.configOpenloopRamp(0);
  }

  public void smoothTankDrive(double leftSpeed, double rightSpeed) {
    setsmooth();
    tankDrive(leftSpeed, rightSpeed);
  }

  public void smoothArcadeDrive(double leftSpeed, double rightSpeed) {
    setsmooth();
    arcadeDrive(leftSpeed, rightSpeed);
  }

  public void jamieDrive(double leftSpeed, double rightSpeed) {
    //setsmooth();
    if (leftSpeed < -0.02) {
      leftTalon().set(ControlMode.PercentOutput, -speedfactor*0.5);
      rightTalon().set(ControlMode.PercentOutput, speedfactor*0.5);
      SmartDashboard.putString("Direction", "Right");
    }

    else if (leftSpeed > 0.02) {
      leftTalon().set(ControlMode.PercentOutput, speedfactor*0.5);
      rightTalon().set(ControlMode.PercentOutput, -speedfactor*0.5);
      SmartDashboard.putString("Direction", "Left");
    } else {
      leftTalon().set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
      rightTalon().set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
      SmartDashboard.putString("Direction", "Straight");
    }
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    robotLeftTalon.set(ControlMode.PercentOutput, -speedfactor * leftSpeed);
    robotRightTalon.set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
    
  }

  public void driveToScoringArea(){
  // ultrasonicRange = ultrasonic.averageRange();
  // if(ultrasonicCheck = false){
  // robotLeftTalon.set(ControlMode.PercentOutput, 0.5);
  // robotRightTalon.set(ControlMode.PercentOutput, 0.5);
  // ultrasonicCheck = true;
  // }
  // else if(ultrasonicCheck = true && ultrasonicRange < 5){
  // robotLeftTalon.set(ControlMode.PercentOutput, 0);
  // robotRightTalon.set(ControlMode.PercentOutput, 0);
  // }
  }

  public void arcadeDrive(double leftSpeed, double rightSpeed) {
    leftTalon().set(ControlMode.PercentOutput, -speedfactor * leftSpeed); 
    rightTalon().set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
  }

  public void autoDrive() {
    leftTalon().set(ControlMode.PercentOutput, autospeedfactor);
    rightTalon().set(ControlMode.PercentOutput, -autospeedfactor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveLeftGyro() {
    SmartDashboard.putString("driveleft", "");
    leftTalon().set(ControlMode.PercentOutput, -.5);
    rightTalon().set(ControlMode.PercentOutput, .5);
  }

  public void driveRightGyro() {
    leftTalon().set(ControlMode.PercentOutput, .5);
    rightTalon().set(ControlMode.PercentOutput, -.5);
  }
  public void driveLeft() {
    SmartDashboard.putString("driveleft", "");
    leftTalon().set(ControlMode.PercentOutput, -autospeedfactor);
    rightTalon().set(ControlMode.PercentOutput, autospeedfactor);
  }

  public void driveRight() {
    leftTalon().set(ControlMode.PercentOutput, autospeedfactor);
    rightTalon().set(ControlMode.PercentOutput, -autospeedfactor);
  }
  public void slowDriveRight() {
    leftTalon().set(ControlMode.PercentOutput, 0.17);
    rightTalon().set(ControlMode.PercentOutput, -0.17);
  }

  public void slowDriveLeft() {
    leftTalon().set(ControlMode.PercentOutput, -0.17);
    rightTalon().set(ControlMode.PercentOutput, 0.17);
  }
  public void stopDrive() {
    leftTalon().set(ControlMode.PercentOutput, 0);
    rightTalon().set(ControlMode.PercentOutput, 0);
    SmartDashboard.putString("stoprobot", "");
  }

  public void inchforward(double speed, double leftultrasonic, double rightultrasonic) {
    //  if (leftultrasonic >=54 && rightultrasonic >=54){
    //   leftTalon().set(ControlMode.PercentOutput, speed);
    //   rightTalon().set(ControlMode.PercentOutput, speed);
    //  }
    if (Math.abs(leftultrasonic - rightultrasonic) <= 6){
      leftTalon().set(ControlMode.PercentOutput, speed);
      rightTalon().set(ControlMode.PercentOutput, speed);
    }else if (rightultrasonic > leftultrasonic){
      driveLeft();
    }
    else if (rightultrasonic < leftultrasonic){
      driveRight();
    }
    else {
      leftTalon().set(ControlMode.PercentOutput, speed);
      rightTalon().set(ControlMode.PercentOutput, speed);
    }
  }


  public String getVisionAction(){
    table = inst.getTable("Vision");
    myEntry = table.getEntry("Action");
    return myEntry.getString("None");
  }
  public boolean getTargetFound(){
    table = inst.getTable("Vision");
    myEntry = table.getEntry("Target Found");
    return myEntry.getBoolean(false);
  }
  public double inchesToTicks(double inch) {
    double input = inch * 4096;
    double answer = input / circumfrence;
    SmartDashboard.putNumber("TargetPose", answer);
    return answer;
  }

  public void goforwards(boolean reduceSpeed) {
    double speed = 0.5;
    if (reduceSpeed) {
      speed = 0.23;
    }
    leftTalon().set(ControlMode.PercentOutput, speed);
    rightTalon().set(ControlMode.PercentOutput, speed);
  }

  public void gobackwards() {
    leftTalon().set(ControlMode.PercentOutput, -autospeedfactor);
    rightTalon().set(ControlMode.PercentOutput, -autospeedfactor);
  }

  public void resetEncoders(){
    robotLeftTalon.setSelectedSensorPosition(0);
    robotRightTalon.setSelectedSensorPosition(0);
  }

  public void writeEncoderValues() {
    SmartDashboard.putNumber("Encoder Right Value", rightTalon().getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Left Value", leftTalon().getSelectedSensorPosition());
  }

  public void endPathFollower() {
    pathFollowerDone = true;
    stopDrive();
  }

  public boolean isPathFollowerDone() {
    return pathFollowerDone;
  }
  public double getLeftUltrasonic() {
    table = inst.getTable("HC-SR04");
    myEntry = table.getEntry("Left Distance");
    double value = myEntry.getDouble(0);
    if (value > 40) {
      // value = 0.0;
    }
    return value;
  }

  public double getRightUltrasonic() {
    table = inst.getTable("HC-SR04");
    myEntry = table.getEntry("Right Distance");
    double value = myEntry.getDouble(0);
    if (value > 40) {
      // value = 0.0;
    }
    return value;
  }
  public double getVisionArea() {
    table = inst.getTable("Vision");
    myEntry = table.getEntry("Area");
    double value = myEntry.getDouble(0);
    return value;
  }
}
