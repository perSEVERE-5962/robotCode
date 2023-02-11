// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor;

public class LineDetector extends SubsystemBase {
  private ColorSensor Color_Sensor;
  private final String red = "Red";
  private final String blue = "Blue";
  private final String Working = "Working";

  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor_read_rgb_values = new ColorSensorV3(i2cPort);
  // Color detectedRGB = m_colorSensor_read_rgb_values.getColor();
  /** Creates a new LineDetector2. */
  public LineDetector() {
    Color_Sensor = new ColorSensor();
  }

  public boolean Sensing_Color() {
    SmartDashboard.putString("Color Name", Color_Sensor.getColor());
    SmartDashboard.putString("Hex Value", Color_Sensor.getHex().toString());
    if (Color_Sensor.getColor() == red || Color_Sensor.getColor() == blue) {
      SmartDashboard.putString("Testing_for_Color", Working);
    }
    // replace blue and red strings with the value as strings
    // robot.stop();
    // robot.move(x);
    // }
    // SmartDashboard.putString("Testing_with_Color", color_not_detected);
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
