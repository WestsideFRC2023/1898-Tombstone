// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.arm.ArmToPosition;

public final class Autos
{
  private static PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
  private static PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
  private static PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);

  public static CommandBase OuttakePlusShortDrive() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Outtake And Short Drive", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new ArmToPosition(ArmConstants.BASE_HIGH_CONSTANT, ArmConstants.WRIST_HIGH_CONSTANT),
      new InstantCommand(() -> RobotContainer.intake.outtake()),
      new ArmToPosition(ArmConstants.BASE_STOW_POS, ArmConstants.WRIST_STOW_POS),
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      driveOnCS,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase OuttakePlusLongDrive() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Outtake And Long Drive", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new ArmToPosition(ArmConstants.BASE_HIGH_CONSTANT, ArmConstants.WRIST_HIGH_CONSTANT),
      new InstantCommand(() -> RobotContainer.intake.outtake()),
      new ArmToPosition(ArmConstants.BASE_STOW_POS, ArmConstants.WRIST_STOW_POS),
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      driveOnCS,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase outtakePlusBalance() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Outtake And Balance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new ArmToPosition(ArmConstants.BASE_HIGH_CONSTANT, ArmConstants.WRIST_HIGH_CONSTANT),
      new InstantCommand(() -> RobotContainer.intake.outtake()),
      new ArmToPosition(ArmConstants.BASE_STOW_POS, ArmConstants.WRIST_STOW_POS),
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      driveOnCS,
      // new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }


  /* Example static factory for an autonomous command. */
  public static CommandBase doNothing() { return new InstantCommand();}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
    
  }


  //Copy paste from old robot
  private static PPSwerveControllerCommand makeSwerveControllerCommand(PathPlannerTrajectory traj){
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    return new PPSwerveControllerCommand(
      traj, 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController,
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);
  }

  private static Pose2d getInitialPose(PathPlannerTrajectory traj){
    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
    PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), Alliance.Red) :
    traj.getInitialState();

    return new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
  }

  
}
