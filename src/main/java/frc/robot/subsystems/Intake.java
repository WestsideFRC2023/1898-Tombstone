// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private VictorSPX intakeMotor;
  
  /** Creates a new Intake. */
  public Intake()
  {
    intakeMotor = new VictorSPX(IntakeConstants.kSparkMaxPort);
  }

  public void pickUp()
  {
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }
  
  public void outtake()
  {
    intakeMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop()
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void hold() {
    intakeMotor.set(ControlMode.PercentOutput, 0.04);
  }

  // public double getSpeed()
  // {
  //   return intakeMotor.getEncoder().getVelocity();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
