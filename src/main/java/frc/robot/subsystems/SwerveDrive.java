// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;;

public class SwerveDrive extends SubsystemBase {
  // swerve module locations
  private final Translation2d frontLeftLocation = new Translation2d(
    SwerveDriveConstants.FL_X_LOCATION, 
    SwerveDriveConstants.FL_Y_LOCATION);
  private final Translation2d backLeftLocation = new Translation2d(
    SwerveDriveConstants.BL_X_LOCATION, 
    SwerveDriveConstants.BL_Y_LOCATION);
  private final Translation2d backRightLocation = new Translation2d(
    SwerveDriveConstants.BR_X_LOCATION, 
    SwerveDriveConstants.BR_Y_LOCATION);
  private final Translation2d frontRightLocation = new Translation2d(
    SwerveDriveConstants.FR_X_LOCATION, 
    SwerveDriveConstants.FR_Y_LOCATION);

  // swerve modules
  private SwerveModule frontLeftSwerveModule = new SwerveModule(
    new CANSparkMax(SwerveDriveConstants.FL_DRIVE_CANID, MotorType.kBrushless),
    new CANSparkMax(SwerveDriveConstants.FL_STEER_CANID, MotorType.kBrushless));

  private SwerveModule backLeftSwerveModule = new SwerveModule(
    new CANSparkMax(SwerveDriveConstants.BL_DRIVE_CANID, MotorType.kBrushless),
    new CANSparkMax(SwerveDriveConstants.BL_STEER_CANID, MotorType.kBrushless));
  
  private SwerveModule backRightSwerveModule = new SwerveModule(
    new CANSparkMax(SwerveDriveConstants.BR_DRIVE_CANID, MotorType.kBrushless),
    new CANSparkMax(SwerveDriveConstants.BR_STEER_CANID, MotorType.kBrushless));
  
  private SwerveModule frontRightSwerveModule = new SwerveModule(
    new CANSparkMax(SwerveDriveConstants.FR_DRIVE_CANID, MotorType.kBrushless),
    new CANSparkMax(SwerveDriveConstants.FR_STEER_CANID, MotorType.kBrushless));

  // state of all swerve modules
  private SwerveModuleState[] swerveModuleStates;

  // builds kinematics module
  // the order that each swerve module is created needs to be followed in calculations
  private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    backLeftLocation,
    backRightLocation,
    frontRightLocation);
  
  // implement NAVX
  private final AHRS NavX = new AHRS(Port.kMXP);
  // build odometry module
  private final SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(
    driveKinematics, 
    NavX.getRotation2d(), 
    new SwerveModulePosition[] {
      frontLeftSwerveModule.getModulePosition(),
      backLeftSwerveModule.getModulePosition(),
      backRightSwerveModule.getModulePosition(),
      frontRightSwerveModule.getModulePosition()
    });
  
  // robot pose
  public Pose2d drivePose;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // sets the offsets of the absolute encoder
    frontLeftSwerveModule.setAbsEncoderOffset(SwerveDriveConstants.FL_ABS_OFFSET);
    backLeftSwerveModule.setAbsEncoderOffset(SwerveDriveConstants.BL_ABS_OFFSET);
    backRightSwerveModule.setAbsEncoderOffset(SwerveDriveConstants.BR_ABS_OFFSET);
    frontRightSwerveModule.setAbsEncoderOffset(SwerveDriveConstants.BR_ABS_OFFSET);
    
    //TODO: do initial setup of NavX
  }
  /**
   * controls the swerve modules of the drivetrain to try to result a desired drivetrain state
   * @param desiredChassisState the desired state of the drivetrain
   */
  public void drive(ChassisSpeeds desiredChassisState) {
    swerveModuleStates = driveKinematics.toSwerveModuleStates(desiredChassisState);
    SwerveDriveKinematics.desaturateWheelSpeeds( // sets cap on wheel speed
      swerveModuleStates, 
      Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FPS));

    // tells modules where to go
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    backLeftSwerveModule.setDesiredState(swerveModuleStates[1]);
    backRightSwerveModule.setDesiredState(swerveModuleStates[2]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * updates the odometry for the field relative position of the robot
   */
  public Pose2d updateOdometry() {
    return driveOdometry.update(
      NavX.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeftSwerveModule.getModulePosition(),
        backLeftSwerveModule.getModulePosition(),
        backRightSwerveModule.getModulePosition(),
        frontRightSwerveModule.getModulePosition()
    });
  }

  /**
   * returns the current heading of drivetrain
   * @return heading of drivetrain
   */
  public Rotation2d getHeading() {
    return NavX.getRotation2d();
  }

  @Override
  public void periodic() {
    drivePose = this.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
