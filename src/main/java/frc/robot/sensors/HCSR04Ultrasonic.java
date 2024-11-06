/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Creates a HC-SR04 ultrasonic
 * pingChannel = TRIG
 * echoChannel = ECHO
 */
public class HCSR04Ultrasonic {  
    private Ultrasonic hcsr04 = new Ultrasonic(0,1); //TODO: put these in constants
    private boolean loggingEnabled = false;
 
    public HCSR04Ultrasonic() {
        hcsr04.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
    }

    /**
     * Retrieves the sensors range in inches
     * @return the range in inches
     */
    public double getRange() {
        log("HCSR04 Ultrasonic", hcsr04.getRangeInches());
        return hcsr04.getRangeInches();
    }

    private void log (String key, double value) {
        if (loggingEnabled)
            SmartDashboard.putNumber(key, value);
    }

}