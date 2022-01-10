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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drive extends SubsystemBase {
  private WPI_TalonSRX robotLeftTalon;
  private WPI_VictorSPX robotLeftVictor;
  private WPI_TalonSRX robotRightTalon;
  private WPI_VictorSPX robotRightVictor;

  private final double speedfactor = 1;
  private final double autospeedfactor = 0.275; 

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setsmooth() {
    robotLeftTalon.configOpenloopRamp(0.7);
    robotLeftTalon.configClosedloopRamp(0);
    robotRightTalon.configOpenloopRamp(0.7);
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

  public void tankDrive(double leftSpeed, double rightSpeed) {
    robotLeftTalon.set(ControlMode.PercentOutput, -speedfactor * leftSpeed);
    robotRightTalon.set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
  }

  public void arcadeDrive(double leftSpeed, double rightSpeed) {
    leftTalon().set(ControlMode.PercentOutput, -speedfactor * leftSpeed); 
    rightTalon().set(ControlMode.PercentOutput, -speedfactor * rightSpeed);
  }

  public void autoDrive() {
    leftTalon().set(ControlMode.PercentOutput, autospeedfactor);
    rightTalon().set(ControlMode.PercentOutput, -autospeedfactor);
  }

  public void stopDrive() {
    leftTalon().set(ControlMode.PercentOutput, 0);
    rightTalon().set(ControlMode.PercentOutput, 0);
  }

  public void resetGyro() {
    ahrs.reset();
  }

  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  public void resetEncoders(){
    robotLeftTalon.setSelectedSensorPosition(0);
    robotRightTalon.setSelectedSensorPosition(0);
  }

}
