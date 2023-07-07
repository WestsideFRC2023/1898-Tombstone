// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
  }

  public static class IntakeConstants
  {
    public static final int kSparkMaxPort = 61;
  }

  public static class ArmConstants {
    public static final int CIM_MASTER_CANID = 50;
    public static final int CIM_SLAVE_CANID = 51;
    public static final int WRIST_CANID = 60;
    public static final int WRIST_ABS_ENC_ID = 0;

    public static final double BASE_RATIO = 80;
    public static final double WRIST_RATIO = 60;

    public static final double BASE_KP = 0;
    public static final double BASE_KI = 0;
    public static final double BASE_KD = 0;

    public static final int BASE_MAX_V = 0;
    public static final int BASE_MAX_A = 0;

    public static final double BASE_STOW_POS = 0; //absolute encoder position
    public static final double WRIST_STOW_POS = 0;
    public static final double WRIST_ERROR_THRESHOLD = 0;
    public static final double BASE_ERROR_THRESHOLD = 0;
    
    public static final double WRIST_kF = 0;
    public static final double WRIST_kP = 0;
    public static final double WRIST_kI = 0;
    public static final double WRIST_kD = 0;

    public static final double WRIST_MAX_V = 0;
    public static final double WRIST_MAX_A = 0;
    public static final int WRIST_CURVE_STR = 0;
    
    public static final double WRIST_HIGH_CONSTANT = 0;
    public static final double BASE_HIGH_CONSTANT = 0;

    public static final double WRIST_MID_CONE_CONSTANT = 0;
    public static final double BASE_MID_CONE_CONSTANT = 0;
    
    public static final double WRIST_MID_CUBE_CONSTANT = 0;
    public static final double BASE_MID_CUBE_CONSTANT = 0;

    public static final double WRIST_INTAKE_CONE_CONSTANT = 0;
    public static final double BASE_INTAKE_CONE_CONSTANT = 0;
    
    public static final double WRIST_INTAKE_CUBE_CONSTANT = 0;
    public static final double BASE_INTAKE_CUBE_CONSTANT = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 20;
    public static final int kFrontRightDrivingCanId = 40;
    public static final int kRearRightDrivingCanId = 30;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 41;
    public static final int kRearRightTurningCanId = 31;

    public static final boolean kGyroReversed = false;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 10;
    public static final int RIGHT_FRONT_DRIVE_ID = 40;
    public static final int LEFT_BACK_DRIVE_ID = 20;
    public static final int RIGHT_BACK_DRIVE_ID = 30;
    
    public static final int LEFT_FRONT_TURN_ID = 11;
    public static final int RIGHT_FRONT_TURN_ID = 41;
    public static final int LEFT_BACK_TURN_ID = 21;
    public static final int RIGHT_BACK_TURN_ID = 31;
    
    // public static final int RIGHT_FRONT_CANCODER_ID_threncID = 0;
    // public static final int LEFT_FRONT_CANCODER_ID_threncID = 1;
    // public static final int RIGHT_BACK_CANCODER_ID_threncID = 2;
    // public static final int LEFT_BACK_CANCODER_ID_threncID = 3;

    //public static final SerialPort PIGEON_ID = SerialPort();

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = 84.96;
    public static final double RIGHT_FRONT_OFFSET = 326.64; //299.88
    public static final double LEFT_BACK_OFFSET = 82.14; //254.52
    public static final double RIGHT_BACK_OFFSET = 24.19; //277.92

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.08;
    public static final double TURN_MOTOR_GEAR_RATIO =  9424/203;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    
    public static final double KP_TURNING = 0.5;

    public static final double DRIVETRAIN_MAX_SPEED = 4.6;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED * 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED /1.75;
    public static final double TELE_DRIVE_FAST_SPEED = DRIVETRAIN_MAX_SPEED * 0.75;
    public static final double TELE_DRIVE_SLOW_SPEED = DRIVETRAIN_MAX_SPEED * 0.2;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 1.5;
    public static final double deadbandValue = 0.25;

    //Auton constraints
    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    public static final double AUTO_kP_FRONT = 0.4; //0.4
    public static final double AUTO_kP_SIDE = 0.4;
    public static final double AUTO_kP_TURN = 0; //2.4

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.009;
  }
}
