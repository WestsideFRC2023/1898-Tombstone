package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.SwerveModuleConstants;

// Class that is used to represent swerve modules, contains motor control and encoder logic
// uses rotation2d for input from encoders
public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;
    private DutyCycleEncoder absEncoder;

    private PIDController drivePID;
    private SimpleMotorFeedforward driveFF;

    // steer motor
    private ProfiledPIDController steerPID;

    // absolute encoder
    private double absEncoderOffset; // offset of the absolute encoder
    private Rotation2d absEncoderRot2d;

    // desired state
    private SwerveModuleState desiredState;
    /**
     * 
     * @param driveMotor
     * @param steerMotor
     * @param encoder
     */
    public SwerveModule(CANSparkMax inDriveMotor, CANSparkMax inSteerMotor, DutyCycleEncoder encoder) {
        // stores inputs
        driveMotor = inDriveMotor;
        steerMotor = inSteerMotor;
        absEncoder = encoder;

        // drive motor setup
        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        driveMotor.getEncoder().setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_VELOCITY_RATIO);
        drivePID = new PIDController(
            SwerveModuleConstants.DRIVE_MOTOR_PID_KP, 
            SwerveModuleConstants.DRIVE_MOTOR_PID_KI, 
            SwerveModuleConstants.DRIVE_MOTOR_PID_KD);
        driveFF = new SimpleMotorFeedforward(
            SwerveModuleConstants.DRIVE_MOTOR_FF_KS,
            SwerveModuleConstants.DRIVE_MOTOR_FF_KA,
            SwerveModuleConstants.DRIVE_MOTOR_FF_KV);

        // steer motor setup
        steerMotor.setSmartCurrentLimit(SwerveModuleConstants.STEER_MOTOR_CURRENT_LIMIT);
        steerMotor.setVoltage(SwerveModuleConstants.STEER_MOTOR_VOLTAGE);
        steerMotor.getEncoder().setPosition(this.getAbsEncoderRot2d().getRotations()); // aligns relative encoder with absolute encoder
        steerMotor.getEncoder().setPositionConversionFactor(1/SwerveModuleConstants.STEER_MOTOR_GEARREDUCTION);
        steerPID = new ProfiledPIDController(
            SwerveModuleConstants.STEER_MOTOR_PID_KP, 
            SwerveModuleConstants.STEER_MOTOR_PID_KI, 
            SwerveModuleConstants.STEER_MOTOR_PID_KD,
            new TrapezoidProfile.Constraints(
                SwerveModuleConstants.STEER_MOTOR_TMP_MAXVELOCITY, 
                SwerveModuleConstants.STEER_MOTOR_TMP_MAXACCELERATION));

        // encoder setup
        absEncoder.setDutyCycleRange(
            SwerveModuleConstants.ABS_ENCODER_DUTYCYCLE_MIN, 
            SwerveModuleConstants.ABS_ENCODER_DUTYCYCLE_MAX);
        
    }

    // Absolute encoder
    /**
     * sets the offset of the absolute encoder
     * @param offset of absolute encoder
     */
    public void setAbsEncoderOffset(double offset) {
        absEncoderOffset = offset;
        absEncoder.setPositionOffset(absEncoderOffset);
    }

    /**
     * This should only be used on startup, use the relative encoder for all other times
     * @return value of the absolute encoder, accounting for rollovers
     */
    public Rotation2d getAbsEncoderRot2d() { // startup angle, zero is forward
        absEncoderRot2d = Rotation2d.fromRotations((absEncoder.get()) % 1);
        return absEncoderRot2d; // startup angle
    }

    // Relative steer encoder
    /**
     * @return Rotation2d of the angle of the wheel through the relative encoder on the motor, accounting for rollover and gear ratio
     */
    public Rotation2d getRelSteerEncoderRot2d() {
        return Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() % 1);
    }

    // Module State
    /**
     * set the desired state of the module from the drivetrain
     * @param inModuleState desired state of the module
     */
    public void setDesiredState(SwerveModuleState inModuleState) {
        desiredState = inModuleState;
    }

    /**
     * uses a profiled PID Controller to control the steer motor to go the desired angle using a trapezoidal motion profile and a PID Controller
     * <p>uses a feedforward Controller and a PID Controller to control the drive motor to go at a desired speed
     */
    public void goToState() {
        driveMotor.setVoltage( // sets voltage of drive motor
            drivePID.calculate(
                driveMotor.getEncoder().getVelocity(), Units.metersToFeet(desiredState.speedMetersPerSecond)) // pid
            +
            driveFF.calculate(Units.metersToFeet(desiredState.speedMetersPerSecond))); // feedforward
                
        steerMotor.set( // sets speed of steering motor
            steerPID.calculate(
                this.getRelSteerEncoderRot2d().getRotations(), // current 
                desiredState.angle.getRotations())); // goal
    }
}