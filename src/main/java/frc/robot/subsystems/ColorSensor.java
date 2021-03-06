/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor{
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  //private final double[] tRed = {0.51, 0.35, 0.14};
  //private final double[] tGreen = {0.15, 0.58, 0.26};
  //private final double[] tBlue = {0.11, 0.42, 0.47};
  //private final double[] tYellow = {0.30, 0.57, 0.13};

  private final double[] tRed = {0.35, 0.43, 0.23};
  private final double[] tGreen = {0.15, 0.58, 0.27};
  private final double[] tBlue = {0.16, 0.38, 0.46};
  private final double[] tYellow = {0.35, 0.55, 0.10};
  
  private final double tConfidence = 0.93;

  private final Color colorBlue = ColorMatch.makeColor(tBlue[0], tBlue[1], tBlue[2]);
  private final Color colorGreen = ColorMatch.makeColor(tGreen[0], tGreen[1], tGreen[2]);
  private final Color colorRed = ColorMatch.makeColor(tRed[0], tRed[1], tRed[2]);
  private final Color colorYellow = ColorMatch.makeColor(tYellow[0], tYellow[1], tYellow[2]);

  public void calibrateTargetColors(){
    //NEED TO DO THIS
  }

  public String getColor(){
    m_colorMatcher.addColorMatch(colorRed);
    m_colorMatcher.addColorMatch(colorBlue);
    m_colorMatcher.addColorMatch(colorGreen);
    m_colorMatcher.addColorMatch(colorYellow);

    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    double confidence = match.confidence;

    String colorName = "";

    if (match.color == colorBlue){
      colorName = "Blue";
    } else if (match.color == colorGreen){
      colorName = "Green";
    } else if (match.color == colorRed){
      colorName = "Red";
    } else if (match.color == colorYellow){
      colorName = "Yellow";
    } else {
      colorName = "None Detected";
    }

    if (confidence < tConfidence){
      colorName = "Confidence Low";
    } 

    return (colorName);
  }

  public void testColor(){
    Color detectedColor = m_colorSensor.getColor();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  public ColorSensor() {

  }

  /*
  @Override
  public void periodic() {
    //testColor();
  }
  */
}
