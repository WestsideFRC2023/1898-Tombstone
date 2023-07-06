// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax baseMotor;
  private CANSparkMax slaveMotor;
  private CANSparkMax wristMotor;

  private SparkMaxPIDController basePID;
  private SparkMaxAbsoluteEncoder baseAbsEncoder;
  
  private SparkMaxPIDController wristPID;
  private SparkMaxAbsoluteEncoder wristAbsEncoder;

  public double baseTarget;
  public double wristTarget;

  /** Creates a new Arm. */
  public Arm() {
    //set up motors
    //brushed (CIMs)
    baseMotor = new CANSparkMax(ArmConstants.CIM_MASTER_CANID, MotorType.kBrushed); 
    slaveMotor = new CANSparkMax(ArmConstants.CIM_SLAVE_CANID, MotorType.kBrushed);
    wristMotor = new CANSparkMax(ArmConstants.WRIST_CANID, MotorType.kBrushless);
    baseAbsEncoder = baseMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristAbsEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);


    //configure CANSparkMax motors
    baseMotor.setIdleMode(IdleMode.kBrake);
    baseMotor.setSmartCurrentLimit(20,30 ); //change if needed
    baseMotor.burnFlash();

    slaveMotor.setIdleMode(IdleMode.kBrake);
    slaveMotor.setSmartCurrentLimit(20, 30); //change if needed
    
    //please help
    slaveMotor.follow(baseMotor, false);
    slaveMotor.burnFlash();
    //smart motion (I believe this is motion magic)
    //please refer to https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    basePID.setP(ArmConstants.BASE_KP);
    basePID.setI(ArmConstants.BASE_KI);
    basePID.setD(ArmConstants.BASE_KD);
    basePID.setSmartMotionMaxVelocity(ArmConstants.BASE_MAX_V, 0);
    basePID.setSmartMotionMaxAccel(ArmConstants.BASE_MAX_A, 0);

    //Configure Neo Motor (wrist)
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(20, 30);
    wristMotor.burnFlash();

    //smart motion (I believe this is motion magic)
    //please refer to https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    wristPID.setP(ArmConstants.BASE_KP);
    wristPID.setI(ArmConstants.BASE_KI);
    wristPID.setD(ArmConstants.BASE_KD);
    wristPID.setSmartMotionMaxVelocity(ArmConstants.BASE_MAX_V, 0);
    wristPID.setSmartMotionMaxAccel(ArmConstants.BASE_MAX_A, 0);

    //OLD Code for the wrist joint with a falcon
    // //configure TalonFX motor (wrist)
    // wristMotor.setSelectedSensorPosition(ArmConstants.WRIST_START_POS);
    // wristMotor.setNeutralMode(NeutralMode.Brake);
    // //motion magic
    // wristMotor.config_kF(0, ArmConstants.WRIST_kF);
    // wristMotor.config_kP(0, ArmConstants.WRIST_kP);
    // wristMotor.config_kI(0, ArmConstants.WRIST_kI);
    // wristMotor.config_kD(0, ArmConstants.WRIST_kD);

    // wristMotor.configMotionCruiseVelocity(ArmConstants.WRIST_MAX_V);
    // wristMotor.configMotionAcceleration(ArmConstants.WRIST_MAX_A);
    // wristMotor.configMotionSCurveStrength(ArmConstants.WRIST_CURVE_STR);
    
    // wristEncInput = new DigitalInput(ArmConstants.WRIST_ABS_ENC_ID);
    // wristAbsEncoder = new DutyCycleEncoder(wristEncInput);
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
    basePID.setReference(baseTarget, CANSparkMax.ControlType.kSmartMotion);
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kSmartMotion);
  }

  /** 
   * checks if wrist is in the arm or not
   */
  public boolean wristStowed() {
    return (Math.abs(wristAbsEncoder.getPosition()  - ArmConstants.WRIST_STOW_POS) 
            < ArmConstants.WRIST_ERROR_THRESHOLD);
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
    return (Math.abs(wristAbsEncoder.getPosition() - wristTarget) 
            < ArmConstants.WRIST_ERROR_THRESHOLD);
    //do we have to involve gear ratio in this somehow
    //please help
  }

  /**
   * stop base and stow wrist
   */
  public void stowWrist() {
    baseMotor.stopMotor();
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kSmartMotion);
  }

  public void armHigh()
  {
    setTargets(ArmConstants.BASE_HIGH_CONSTANT, ArmConstants.WRIST_HIGH_CONSTANT);
    passTargets();
  }

  public void armMidCone()
  {
    setTargets(ArmConstants.BASE_MID_CONE_CONSTANT, ArmConstants.WRIST_MID_CONE_CONSTANT);
    passTargets();
  }

  public void armMidCube()
  {
    setTargets(ArmConstants.BASE_MID_CUBE_CONSTANT, ArmConstants.WRIST_MID_CUBE_CONSTANT);
    passTargets();
  }

  public void armIntakeCone()
  {
    setTargets(ArmConstants.BASE_INTAKE_CONE_CONSTANT, ArmConstants.WRIST_INTAKE_CONE_CONSTANT);
    passTargets();
  }

  public void armIntakeCube()
  {
    setTargets(ArmConstants.BASE_INTAKE_CUBE_CONSTANT, ArmConstants.WRIST_INTAKE_CUBE_CONSTANT);
    passTargets();
  }

  public void armStow()
  {
    setTargets(ArmConstants.BASE_STOW_POS, ArmConstants.WRIST_STOW_POS);
    passTargets();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Base Encoder", baseAbsEncoder.getPosition());
    SmartDashboard.putNumber("Base Target", baseTarget);

    SmartDashboard.putNumber("Wrist Encoder", wristAbsEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Target", wristTarget);
  }

}
