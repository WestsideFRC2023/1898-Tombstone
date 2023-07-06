// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Arm arm = new Arm();
  public static final Intake intake = new Intake();
  private SendableChooser<String> autoChooser = new SendableChooser<>();
  

  // XBOX Controller Definition
  public static final XboxController driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  public static final XboxController m_opController =
      new XboxController(OperatorConstants.kOpControllerPort);

  // XBOX Button Maps
  private final JoystickButton Intake_LB = new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton Outtake_RB = new JoystickButton(m_opController, XboxController.Button.kRightBumper.value);

  private final JoystickButton ArmHigh = new JoystickButton(m_opController, XboxController.Button.kY.value);
  private final JoystickButton ArmMid = new JoystickButton(m_opController, XboxController.Button.kB.value);
  private final JoystickButton ArmIntake = new JoystickButton(m_opController, XboxController.Button.kA.value);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kA.value);

  // ARM Commands to handle Cone/Cube If Statement
  // If any POV button is pressed, then the arm will go into cube Mode
  // Otherwise the Arm is in Cone Mode
  public CommandBase ArmMid()
  {
    if (m_opController.getPOV() != -1)
    {
      return arm.runOnce(() -> arm.armMidCube());
    }
    else
    {
      return arm.runOnce(() -> arm.armMidCone());
    }
  }
  public CommandBase ArmIntake()
  {
    if (m_opController.getPOV() != -1)
    {
      return arm.runOnce(() -> arm.armIntakeCube());
    }
    else
    {
      return arm.runOnce(() -> arm.armIntakeCone());
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    configureBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings()
  {
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    Intake_LB.onTrue(new InstantCommand(() -> intake.pickUp()));
    Intake_LB.onFalse(new InstantCommand(() -> intake.hold()));
    Outtake_RB.onTrue(new InstantCommand(() -> intake.outtake()));
    Outtake_RB.onFalse(new InstantCommand(() -> intake.stop()));

    ArmHigh.onTrue(new InstantCommand(() -> arm.armHigh()));
    ArmHigh.onFalse(new InstantCommand(() -> arm.armStow()));

    ArmMid.onTrue(ArmMid());
    ArmMid.onFalse(new InstantCommand(() -> arm.armStow()));

    ArmIntake.onTrue(ArmIntake());
    ArmIntake.onFalse(new InstantCommand(() -> arm.armStow()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(); 
  }
}
