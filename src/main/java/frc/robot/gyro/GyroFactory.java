// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorControllerType;

public class GyroFactory {
    /**
     * create the gyro based on the controller type
     *
     * <p>
     * starting with the 2022 season we are switching from CTRE (TalonSRX &
     * VictorSPX) to REV
     * (Spark Max) controllers
     *
     * @return the gyro to use
     */
    public GyroInterface createGyro() {
        return createGyro(MotorControllerType.kREV);
    }

    /**
     * Create a gyro based on the specified controller type
     *
     * @param motorControllerType is one of the types defined in {@link
     *                            frc.robot.Constants.MotorControllerType}
     * @returns an instance of {@link frc.robot.factories.GyroInterface}
     */
    public GyroInterface createGyro(int motorControllerType) {
        GyroInterface gyro;
        switch (motorControllerType) {
            case MotorControllerType.kCTRE:
            case MotorControllerType.kREV:
            case MotorControllerType.kHybrid:
                SmartDashboard.putString("Selected Gyro", "AHRS");
                gyro = new AHRSGyro();
                break;
            case MotorControllerType.kRomi:
                gyro = new RomiGyro();
                SmartDashboard.putString("Selected Gyro", "Romi");
                break;
            default:
                gyro = new AHRSGyro();
                SmartDashboard.putString("Selected Gyro", "Default");
                break;
        }
        return gyro;
    }
}
