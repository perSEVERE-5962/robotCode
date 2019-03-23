package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class placeHatch extends Command {
    // double PLACE_HATCH_POSITION = 682.2;

    // public boolean onTarget(){
    // if(Math.abs(RobotMap.armTalon.getClosedLoopError())<10){
    // return true;
    // }
    // return false;
    // }
    // public void moveToPlaceHatch (){
    // RobotMap.armTalon.set(ControlMode.Position, PLACE_HATCH_POSITION);
    // SmartDashboard.putNumber("target position", PLACE_HATCH_POSITION);
    // }
    public placeHatch() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Robot.armMotor.moveToPlaceHatch();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute(){
        if (Robot.armMotor.isPIDRunning() == false) {
            Robot.armMotor.moveToPlaceHatch();
        }
        Robot.logger.putNumber("Closed Loop Error", RobotMap.armTalon.getClosedLoopError());
    }

    protected boolean isFinished() {

        if (Robot.armMotor.isOnTarget() == true && Robot.armMotor.isPIDRunning() == true) {
            return true;
        } else {
            return false;
        }
    }
      // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
}