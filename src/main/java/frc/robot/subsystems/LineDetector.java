// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor;

public class LineDetector extends SubsystemBase {
  private ColorSensor Color_Sensor;
  private final String red = "red";
  private final String blue = "blue";

  /** Creates a new LineDetector2. */
  public LineDetector() {
    Color_Sensor = new ColorSensor();
  }

  public boolean Sensing_Color() {
    SmartDashboard.putString("Color Value", Color_Sensor.getColor().toString());
    if (Color_Sensor.getColor().toString() == red || Color_Sensor.getColor().toString() == blue) {
      return true;
    }
    // robot.stop();
    // robot.move(x);
    // }
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
