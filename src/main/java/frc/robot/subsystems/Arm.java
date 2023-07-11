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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax baseMotor;
  private CANSparkMax slaveMotor;
  private CANSparkMax wristMotor;

  private SparkMaxPIDController basePID;
  private SparkMaxAbsoluteEncoder baseAbsEncoder;
  
  private SparkMaxPIDController wristPID;
  private RelativeEncoder wristEncoder;

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
    wristEncoder = wristMotor.getEncoder();


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
    basePID = baseMotor.getPIDController();
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
    wristPID = wristMotor.getPIDController();
    wristPID.setP(ArmConstants.WRIST_kP);
    wristPID.setI(ArmConstants.WRIST_kI);
    wristPID.setD(ArmConstants.WRIST_kD);
    wristPID.setSmartMotionMaxVelocity(ArmConstants.BASE_MAX_V, 0);
    wristPID.setSmartMotionMaxAccel(ArmConstants.BASE_MAX_A, 0);
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
    basePID.setReference(baseTarget, CANSparkMax.ControlType.kPosition);
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
  }

  /** 
   * checks if wrist is in the arm or not
   */
  public boolean wristStowed() {
    return (Math.abs(wristEncoder.getPosition()  - ArmConstants.WRIST_STOW_POS) 
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
    return (Math.abs(wristEncoder.getPosition() - wristTarget) 
            < ArmConstants.WRIST_ERROR_THRESHOLD);
    //do we have to involve gear ratio in this somehow
    //please help
  }

  /**
   * stop base and stow wrist
   */
  public void stowWrist() {
    baseMotor.stopMotor();
    wristPID.setReference(wristTarget, CANSparkMax.ControlType.kPosition);
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

    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Target", wristTarget);
  }

}
