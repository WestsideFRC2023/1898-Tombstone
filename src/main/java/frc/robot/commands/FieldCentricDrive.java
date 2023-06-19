// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FieldCentricDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveDrive swerveDrive;

  /**
   * Constructs a command that will control the drivetrain in field-centric mode
   * @param subsystem SwerveDrive subsystem used by this command.
   */
  public FieldCentricDrive(SwerveDrive subsystem) {
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
    swerveDrive.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
          Units.feetToMeters(swerveDrive.MaxSpeed) * RobotContainer.driverController.getLeftX(), 
          Units.feetToMeters(swerveDrive.MaxSpeed) * RobotContainer.driverController.getLeftY(), 
          Units.rotationsToRadians(swerveDrive.MaxTurn) * RobotContainer.driverController.getRightX()),
        swerveDrive.getHeading()));
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
