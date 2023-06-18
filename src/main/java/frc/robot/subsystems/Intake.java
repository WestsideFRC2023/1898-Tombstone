// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private CANSparkMax intakeMotor;
  
  /** Creates a new Intake. */
  public Intake()
  {
    intakeMotor = new CANSparkMax(IntakeConstants.kSparkMaxPort, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(20, 25);
    intakeMotor.burnFlash();
  }

  public void pickUp()
  {
    intakeMotor.set(0.75);
  }
  
  public void outtake()
  {
    intakeMotor.set(-0.75);
  }

  public void stop()
  {
    intakeMotor.set(0);
  }

  public void hold() {
    intakeMotor.set(.004);
  }

  public double getSpeed()
  {
    return intakeMotor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
