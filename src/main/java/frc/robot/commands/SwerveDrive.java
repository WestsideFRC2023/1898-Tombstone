// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      // This is the code that ran for all of Houston District and it worked If needed revert to this
      // RobotContainer.drivetrain.swerveDrive(
      //   RobotContainer.driverController.getLeftY(), 
      //   RobotContainer.driverController.getLeftX(), 
      //   RobotContainer.driverController.getRightX(),
      //   -RobotContainer.driverController.getRightY(),
      //   !RobotContainer.driverController.getRawButton(XboxController.Button.kB.value),
      //   false,
      //   true);
      RobotContainer.drivetrain.drive(
        deadband(-RobotContainer.driverController.getLeftY()), 
        deadband(-RobotContainer.driverController.getLeftX()), 
        deadband(RobotContainer.driverController.getRightX()),
        !RobotContainer.driverController.getRawButton(XboxController.Button.kB.value),
        true);
  }

  public double deadband(double input)
  {
    double deadband = 0.1;
    if (Math.abs(input) < deadband)
    {
      return 0;
    }
    else
    {return input;}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
