package frc.robot.sensors;

public abstract class UltrasonicBase {

	public static final int DEFAULT_HOLD_DISTANCE = 12;
	public static final double DEFAULT_SPEED_CONSTANT = 0.05;
	public static final String DEFAULT_ENABLE_KEY = "Enable Ultrasonic Range Finder";

	int holdDistance = DEFAULT_HOLD_DISTANCE;

	double speedConstant = DEFAULT_SPEED_CONSTANT;

	String enableKey = "";

	public abstract double getRange();

	/**
	 * Tells the robot to drive to a set distance (in inches) from an object using
	 * proportional control.
	 */
	public double getCurrentSpeed() {
		double currentDistance;
		double currentSpeed;

		currentDistance = getRange();
		currentSpeed = (holdDistance - currentDistance) * speedConstant;
		return currentSpeed;
	}

	public void setHoldDistance(int holdDistance) {
		this.holdDistance = holdDistance;
	}

	public int getHoldDistance() {
		return holdDistance;
	}

	public void setSpeedConstant(double speedConstant) {
		this.speedConstant = speedConstant;
	}

	public double getSpeedConstant() {
		return speedConstant;
	}

	public abstract boolean isEnabled();
	public abstract void setBackwards(boolean backwards); // this is only used for testing
	public abstract void reset(); // this is only used for testing
}