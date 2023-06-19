// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
  
  public static class SwerveDriveConstants {
    /* 
      locations of each swerve module
      FL = Front Left
      FR = Front Right
      BL = Back Left
      BR = Back Right

      X = offset from center, + is forward, - is backwards
      Y = offset from center, + is left, - is right
    */    

    public static final double FL_X_LOCATION = 11.75;
    public static final double FL_Y_LOCATION = 11.75;

    public static final double BL_X_LOCATION = -11.75;
    public static final double BL_Y_LOCATION = 11.75;

    public static final double BR_X_LOCATION = -11.75;
    public static final double BR_Y_LOCATION = -11.75;

    public static final double FR_X_LOCATION = 11.75;
    public static final double FR_Y_LOCATION = -11.75;

    // CAN IDs and PWM Control

    public static final int FL_DRIVE_CANID = 10;
    public static final int FL_STEER_CANID = 11;
    public static final int FL_ABS_PWMID = 0;

    public static final int BL_DRIVE_CANID = 20;
    public static final int BL_STEER_CANID = 21;
    public static final int BL_ABS_PWMID = 1;

    public static final int BR_DRIVE_CANID = 30;
    public static final int BR_STEER_CANID = 31;
    public static final int BR_ABS_PWMID = 2;

    public static final int FR_DRIVE_CANID = 40;
    public static final int FR_STEER_CANID = 41;
    public static final int FR_ABS_PWMID = 3;

  }
  public static class SwerveModuleConstants {
    
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final double DRIVE_MOTOR_GEARREDUCTION = 5.08;
    public static final double DRIVE_MOTOR_VELOCITY_RATIO = (1/DRIVE_MOTOR_GEARREDUCTION) * (Math.PI*3);
    public static final double DRIVE_MOTOR_MAX_VOLTAGE = 12;
    // Drive PID
    public static final double DRIVE_MOTOR_PID_KD = 0;
    public static final double DRIVE_MOTOR_PID_KP = 0;
    public static final double DRIVE_MOTOR_PID_KI = 0;
    // Drive FeedForward
    public static final double DRIVE_MOTOR_FF_KS = 0;
    public static final double DRIVE_MOTOR_FF_KV = 0;
    public static final double DRIVE_MOTOR_FF_KA = 0;

    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
    public static final double STEER_MOTOR_GEARREDUCTION = 9424/203;
    public static final double STEER_MOTOR_VOLTAGE = 12;
    // Steer PID
    public static final double STEER_MOTOR_PID_KD = 0;
    public static final double STEER_MOTOR_PID_KP = 0;
    public static final double STEER_MOTOR_PID_KI = 0;
    // Steer Trapezoidal Motion Profile
    public static final double STEER_MOTOR_TMP_MAXVELOCITY = 5;
    public static final double STEER_MOTOR_TMP_MAXACCELERATION = 10;

    public static final double ABS_ENCODER_DUTYCYCLE_MIN = 1/1025;
    public static final double ABS_ENCODER_DUTYCYCLE_MAX = 1024/1025;
  }
}
