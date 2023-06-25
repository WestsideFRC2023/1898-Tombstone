// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax baseMotor;
  private CANSparkMax baseMotor2;
  private WPI_TalonFX wristMotor;

  private SparkMaxPIDController basePID;
  private SparkMaxAbsoluteEncoder baseAbsEncoder;

  public double baseTarget;
  public double wristTarget;

  /** Creates a new Arm. */
  public Arm() {
    //set up motors
    // brushless or brushed?
    //please help
    baseMotor = new CANSparkMax(ArmConstants.CIM_MASTER_CANID, MotorType.kBrushless); 
    baseMotor2 = new CANSparkMax(ArmConstants.CIM_SLAVE_CANID, MotorType.kBrushless);
    wristMotor = new WPI_TalonFX(ArmConstants.FAL_CANID);
    baseAbsEncoder = baseMotor.getAbsoluteEncoder(Type.kDutyCycle);


    //configure CANSparkMax motors
    baseMotor.setIdleMode(IdleMode.kBrake);
    baseMotor.setSmartCurrentLimit(20, 25); //change if needed
    //please help
    baseMotor.burnFlash();

    baseMotor2.setIdleMode(IdleMode.kBrake);
    baseMotor2.setSmartCurrentLimit(20, 25); //change if needed
    //please help
    baseMotor2.follow(baseMotor, false); // invert if needed
    baseMotor2.burnFlash();
    //smart motion (I believe this is motion magic)
    //please refer to https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    basePID.setP(ArmConstants.BASE_KP);
    basePID.setI(ArmConstants.BASE_KI);
    basePID.setD(ArmConstants.BASE_KD);
    basePID.setSmartMotionMaxVelocity(ArmConstants.BASE_MAX_V, 0);
    basePID.setSmartMotionMaxAccel(ArmConstants.BASE_MAX_A, 0);

    //configure TalonFX motor (wrist)
    wristMotor.setSelectedSensorPosition(ArmConstants.WRIST_START_POS);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    //motion magic
    wristMotor.config_kF(0, ArmConstants.WRIST_kF);
    wristMotor.config_kP(0, ArmConstants.WRIST_kP);
    wristMotor.config_kI(0, ArmConstants.WRIST_kI);
    wristMotor.config_kD(0, ArmConstants.WRIST_kD);

    wristMotor.configMotionCruiseVelocity(ArmConstants.WRIST_MAX_V);
    wristMotor.configMotionAcceleration(ArmConstants.WRIST_MAX_A);
    wristMotor.configMotionSCurveStrength(ArmConstants.WRIST_CURVE_STR);
  }

  /**
   * set the target position of the motors
   * @param base target
   * @param wrist target
   */
  public void setTargets(double baseTarget_, double wristTarget_) {
    baseTarget = baseTarget_;
    wristTarget = wristTarget_;
  }

  /**
   * pass the target position to the motors in motion magic/smart motion mode
   */
  public void passTargets() {
    //baseMotor.setVoltage(to a value calculated by the pid in smart motion mode probably);
    //please help
    wristMotor.set(TalonFXControlMode.MotionMagic, wristTarget);
  }

  /** 
   * checks if wrist is in the arm or not
   */
  public boolean wristStowed() {
    return (Math.abs(wristMotor.getSelectedSensorPosition() - ArmConstants.WRIST_STOWED_POS) 
            < ArmConstants.WRIST_ERROR_THRESHOLD);
    //do we have to involve gear ratio in this somehow
    //please help
  }

  /**
   * checks if base is at target Position
   */
  public boolean baseAtTarget() {
    return (Math.abs(baseAbsEncoder.getPosition() - baseTarget) 
            < ArmConstants.BASE_ERROR_THRESHOLD);
    //do we have to involve gear ratio in this somehow
    //please help
  }

  /**
   * checks if wrist is at target Position
   */
  public boolean wristAtTarget() {
    return (Math.abs(wristMotor.getSelectedSensorPosition() - wristTarget) 
            < ArmConstants.WRIST_ERROR_THRESHOLD);
    //do we have to involve gear ratio in this somehow
    //please help
  }

  /**
   * stop base and stow wrist
   */
  public void stowWrist() {
    baseMotor.stopMotor();
    wristMotor.set(TalonFXControlMode.MotionMagic, ArmConstants.WRIST_STOWED_POS);
  }

}
