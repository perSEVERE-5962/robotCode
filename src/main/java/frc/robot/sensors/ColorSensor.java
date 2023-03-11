package frc.robot.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants;

public class ColorSensor {
  private GenericEntry confidenceEntry;
  private GenericEntry hexEntry;
  private String tab = Constants.tabs.kLineDetector;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  String colorString;

  private final Color Blue_range = new Color(30, 91, 133);
  // private final Color Red_range = new Color(0.561, 0.232, 0.114);
  private final Color Red_range = new Color(131, 89, 33);

  public ColorSensor() {
    AddToShuffleboard.add(tab, "Confidence", 0);
    AddToShuffleboard.add(tab, "Hex", "");

    m_colorMatcher.addColorMatch(Blue_range);
    m_colorMatcher.addColorMatch(Red_range);
  }

  public String getColor() {
    Color Detected_Color = m_colorSensor.getColor();
    // String colorString;
    m_colorMatcher.setConfidenceThreshold(.90);
    ColorMatchResult match = m_colorMatcher.matchClosestColor(Detected_Color);
    if (match.color == Blue_range && match.confidence >= 0.8) {
      colorString = "Blue";
    } else if (match.color == Red_range && match.confidence >= 0.8) {
      colorString = "Red";
    } else {
      colorString = "Unknown Color";
    }

    confidenceEntry.setDouble(match.confidence);

    return colorString;
  }

  public Color getHex() {
    final ColorSensorV3 m_HexSensor = new ColorSensorV3(i2cPort);
    Color detectedHex = m_HexSensor.getColor();
    hexEntry.setString(detectedHex.toHexString());
    return detectedHex;
  }
}
