// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
public class RobotCentricDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveDrive swerveDrive;

  /**
   * Constructs a command that will control the drivetrain in robot-centric mode
   * @param subsystem SwerveDrive subsystem used by this command.
   */
  public RobotCentricDrive(SwerveDrive subsystem) {
    swerveDrive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double XHeading = RobotContainer.driverController.getLeftX();
    double YHeading = RobotContainer.driverController.getLeftY();
    double XTurn = RobotContainer.driverController.getRightX();

    XHeading = Math.abs(XHeading) > OperatorConstants.HEADING_DEADBAND ? XHeading : 0;
    YHeading = Math.abs(YHeading) > OperatorConstants.HEADING_DEADBAND ? YHeading : 0;
    XTurn = Math.abs(XTurn) > OperatorConstants.TURN_DEADBAND ? XTurn : 0;

    swerveDrive.drive(
      new ChassisSpeeds(
        Units.feetToMeters(swerveDrive.MaxSpeed) * XHeading, 
        Units.feetToMeters(swerveDrive.MaxSpeed) * YHeading, 
        Units.rotationsToRadians(swerveDrive.MaxTurn) * XTurn));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
