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

    public static final int BASE_KP = 0;
    public static final int BASE_KI = 0;
    public static final int BASE_KD = 0;

    public static final int BASE_MAX_V = 0;
    public static final int BASE_MAX_A = 0;

    public static final int WRIST_START_POS = 0;
    public static final double BASE_START_POS = 0; //absolute encoder position
    public static final int WRIST_STOWED_POS = 0;
    public static final int WRIST_ERROR_THRESHOLD = 0;
    public static final int BASE_ERROR_THRESHOLD = 0;
    
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
}
