package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.ColorSensor;

public class LineDetector {
  private ColorSensor Color_Sensor;
  private final String red = "red";
  private final String blue = "blue";

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
}
