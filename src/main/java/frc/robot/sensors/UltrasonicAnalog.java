// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicAnalog {
	AnalogInput ultrasonic;
	final double valueToInches = 2.3;

	public UltrasonicAnalog(int channel) {
		ultrasonic = new AnalogInput(channel);
	}

	public double getRange() {
		return ultrasonic.getVoltage() * valueToInches;
	}

	public double getVoltage() {
		return ultrasonic.getVoltage();
	}
	public boolean isEnabled() {
		return true;
	}
}

