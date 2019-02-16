// package frc.robot.sensors;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Joystick;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
// import com.ctre.phoenix.motorcontrol.SensorTerm;
// import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FollowerType;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// public class srxMagEncoderPID extends TimedRobot {
// 	/** Hardware */
// 	TalonSRX _leftMaster = new TalonSRX(2);
// 	TalonSRX _rightMaster = new TalonSRX(1);
// 	Joystick _gamepad = new Joystick(0);
	
// 	/** Latched values to detect on-press events for buttons */
// 	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
// 	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
// 	/** Tracking variables */
// 	boolean _firstCall = false;
// 	boolean _state = false;
// 	double _lockedDistance = 0;
// 	double _targetAngle = 0;

// 	@Override
// 	public void robotInit() {
// 		/* Not used in this project */
// 	}
	
// 	@Override
// 	public void teleopInit(){
// 		/* Disable all motor controllers */
// 		_rightMaster.set(ControlMode.PercentOutput, 0);
// 		_leftMaster.set(ControlMode.PercentOutput, 0);

// 		/* Factory Default all hardware to prevent unexpected behaviour */
// 		_rightMaster.configFactoryDefault();
// 		_leftMaster.configFactoryDefault();
		
// 		/* Set Neutral Mode */
// 		_leftMaster.setNeutralMode(NeutralMode.Brake);
// 		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
// 		/** Feedback Sensor Configuration */
		
// 		/* Configure the left Talon's selected sensor as local QuadEncoder */
// 		_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
// 													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
// 													Constants.kTimeoutMs);					// Configuration Timeout

// 		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
// 		_rightMaster.configRemoteFeedbackFilter(_leftMaster.getDeviceID(),					// Device ID of Source
// 												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
// 												Constants.REMOTE_1,							// Source number [0, 1]
// 												Constants.kTimeoutMs);						// Configuration Timeout
		
// 		/* Setup Sum signal to be used for Distance */
// 		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
// 		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
// 		/* Setup Difference signal to be used for Turn */
// 		_rightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, Constants.kTimeoutMs);
// 		_rightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);
		
// 		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
// 		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
// 													Constants.PID_PRIMARY,
// 													Constants.kTimeoutMs);
		
// 		/* Scale Feedback by 0.5 to half the sum of Distance */
// 		_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
// 														Constants.PID_PRIMARY,		// PID Slot of Source 
// 														Constants.kTimeoutMs);		// Configuration Timeout
		
// 		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
// 		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
// 													Constants.PID_TURN, 
// 													Constants.kTimeoutMs);
		
// 		/* Scale the Feedback Sensor using a coefficient */
// 		_rightMaster.configSelectedFeedbackCoefficient(	1,
// 														Constants.PID_TURN, 
// 														Constants.kTimeoutMs);
		
// 		/* Configure output and sensor direction */
// 		_leftMaster.setInverted(false);
// 		_leftMaster.setSensorPhase(true);
// 		_rightMaster.setInverted(true);
// 		_rightMaster.setSensorPhase(true);
		
// 		/* Set status frame periods to ensure we don't have stale data */
// 		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
// 		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
// 		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
// 		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

// 		/* Configure neutral deadband */
// 		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
// 		_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

// 		/* Max out the peak output (for all modes).  
// 		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
// 		 */
// 		_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
// 		_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
// 		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
// 		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

// 		/* FPID Gains for distance servo */
// 		_rightMaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
// 		_rightMaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
// 		_rightMaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
// 		_rightMaster.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
// 		_rightMaster.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
// 		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);

// 		/* FPID Gains for turn servo */
// 		_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
// 		_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
// 		_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
// 		_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
// 		_rightMaster.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
// 		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
			
// 		/* 1ms per loop.  PID loop can be slowed down if need be.
// 		 * For example,
// 		 * - if sensor updates are too slow
// 		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
// 		 * - sensor movement is very slow causing the derivative error to be near zero.
// 		 */
//         int closedLoopTimeMs = 1;
//         _rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
//         _rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

// 		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
// 		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
// 		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
// 		 */
// 		_rightMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

// 		/* Initialize */
// 		_firstCall = true;
// 		_state = false;
// 		zeroSensors();
// 	}
	
// 	@Override
// 	public void teleopPeriodic() {
// 		/* Gamepad processing */
// 		double forward = -1 * _gamepad.getY();
// 		double turn = _gamepad.getTwist();
// 		forward = Deadband(forward);
// 		turn = Deadband(turn);
	
// 		/* Button processing for state toggle and sensor zeroing */
// 		getButtons(btns, _gamepad);
// 		if(btns[2] && !_btns[2]){
// 			_state = !_state; 		// Toggle state
// 			_firstCall = true;		// State change, do first call operation
// 			_targetAngle = _rightMaster.getSelectedSensorPosition(1);
// 			_lockedDistance = _rightMaster.getSelectedSensorPosition(0);
// 		}else if (btns[1] && !_btns[1]) {
// 			zeroSensors();			// Zero Sensors
// 		}
// 		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
		
// 		if(!_state){
// 			if (_firstCall)
// 				System.out.println("This is a Arcade Drive.\n");
			
// 			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
// 			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
// 		}else{
// 			if (_firstCall) {
// 				System.out.println("This is Drive Straight Distance with the Auxiliary PID using the difference between two encoders.");
// 				System.out.println("Servo [-6, 6] rotations while also maintaining a straight heading.\n");
				
// 				/* Determine which slot affects which PID */
// 				_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
// 				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
// 			}
			
// 			/* Calculate targets from gamepad inputs */
// 			double target_sensorUnits = forward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel  + _lockedDistance;
// 			double target_turn = _targetAngle;
			
// 			/* Configured for Position Closed loop on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
// 			_rightMaster.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, target_turn);
// 			_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
// 		}
// 		_firstCall = false;
// 	}
	
// 	/* Zero quadrature encoders on Talons */
// 	void zeroSensors() {
// 		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
// 		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
// 		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
// 	}
	
// 	/** Deadband 5 percent, used on the gamepad */
// 	double Deadband(double value) {
// 		/* Upper deadband */
// 		if (value >= +0.05) 
// 			return value;
		
// 		/* Lower deadband */
// 		if (value <= -0.05)
// 			return value;
		
// 		/* Outside deadband */
// 		return 0;
// 	}
	
// 	/** Gets all buttons from gamepad */
// 	void getButtons(boolean[] btns, Joystick gamepad) {
// 		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
// 			btns[i] = gamepad.getRawButton(i);
// 		}
//     }
// }