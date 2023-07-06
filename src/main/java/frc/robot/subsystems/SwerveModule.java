// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private PearadoxSparkMax driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;

  private SparkMaxAbsoluteEncoder absoluteEncoder;



  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
                      double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      driveMotor = new PearadoxSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kBrake, 25, turnMotorReversed);
      absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
      absoluteEncoder.setPositionConversionFactor(360);
      
      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  private static double coterminal(double rot) {
    final double full = Math.signum(rot) *2 * Math.PI;
    while(rot > Math.PI || rot < - Math.PI) rot -= full;
    return rot;

  }
  
  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return coterminal(turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getPosition(); //between 0-1
    angle *= 360;
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI / 180);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState);
    
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] Desired State", desiredState.toString());

    SmartDashboard.putNumber("Swerve " + driveMotor.getDeviceId() + " Abs Encoder", getAbsoluteEncoderAngle());
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
    SmartDashboard.putNumber("Swerve [" + driveMotor.getDeviceId() + "] smps", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve [" + driveMotor.getDeviceId() + "] max speed", SwerveConstants.DRIVETRAIN_MAX_SPEED);
    SmartDashboard.putNumber("Swerve [" + driveMotor.getDeviceId() + "] speed", desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);

    // driveController.setReference(
    //   desiredState.speedMetersPerSecond,
    //   ControlType.kVelocity,
    //   0,
    //   feedforward.calculate(desiredState.speedMetersPerSecond));
  }

  public void setAngle(SwerveModuleState desiredState){
   // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    // lastAngle = angle;
  }
  
  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public SparkMaxAbsoluteEncoder getAbsoluteEncoder () {
    return absoluteEncoder;
  }
}
