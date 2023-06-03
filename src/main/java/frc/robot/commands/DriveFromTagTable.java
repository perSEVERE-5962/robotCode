package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

public class DriveFromTagTable extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;

  public DriveFromTagTable(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = 0;
    double ySpeed = 0;
    double turningSpeed = 0;
    // Rather, read from network tables
    NetworkTableEntry direction = NetworkTableInstance.getDefault().getEntry("TagMove");
    NetworkTableValue dirVal = direction.getValue();
    String dirString = "";
    if (dirVal.isString()) {
      dirString = dirVal.getString();
    }
    if (dirString.equals("f")) {
      ySpeed = -0.5;
    } else if (dirString.equals("l")) {
      turningSpeed = 0.5;
    } else if (dirString.equals("r")) {
      turningSpeed = -0.5;
    }

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed * -1, turningSpeed * -1);

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
