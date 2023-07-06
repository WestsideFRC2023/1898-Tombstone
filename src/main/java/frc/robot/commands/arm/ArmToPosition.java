package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmToPosition extends CommandBase{
  
  Arm mArm;

  double baseTarget;
  double wristTarget;

  Timer timer = new Timer();
  boolean timerStarted = false;

  double delay;

  /**
   * move arm to the designated position with motion magic
   * @param arm an arm subsystem
   * @param basePos target basePos of the command
   * @param wristPos target wristPos of the command
   */
  public ArmToPosition(double basePos, double wristPos) {
    mArm = RobotContainer.arm;

    baseTarget = basePos;
    wristTarget = wristPos;

    delay = 0.25;
  }

  /**
   * move arm to the designated position with motion magic and a custon delay time
   * @param arm an arm subsystem
   * @param basePos target basePos of the command
   * @param wristPos target wristPos of the command
   * @param delayTime the time interval between arm reaches target and command terminates
   */
  public ArmToPosition(double basePos, double wristPos, double delayTime) {
    mArm = RobotContainer.arm;

    baseTarget = basePos;
    wristTarget = wristPos;

    delay = delayTime;
  }

  // reset timer and stop so the command doesn't end prematurely
  @Override
  public void initialize() {
    mArm.setTargets(baseTarget, wristTarget);
    timer.reset();
    timer.stop();
    timerStarted = false;
  }

  @Override
  public void execute() {

    /*
     * If the Base is at the target, send the targets (wrist and base) to the PID controller
     * If the wrist is stowed, send the base to the base target to the PID controller
     * Otherwise, Stow the wrist
    */

    if (mArm.baseAtTarget())
    {
      mArm.passTargets();
    }
    else if (mArm.wristStowed())
    {
      mArm.setTargets(baseTarget, ArmConstants.WRIST_STOW_POS);
      mArm.passTargets();
    }
    else
    {
      mArm.stowWrist();
    }

    // start timer once arm reaches target
    if (!timerStarted && (mArm.baseAtTarget() && mArm.wristAtTarget())) {
        timerStarted = true;
        timer.reset();
        timer.start();
    }
  }

  // end command once delay is over
  @Override
  public boolean isFinished()
  {
    /*
     * When the arm reaches it's taget on both joints, a timer starts
     * When the timer passes the set delay variable, other commands can run. 
    */
    return timer.hasElapsed(delay);
  }

  @Override
  public void end(boolean isFinished) 
  {
    //Reports when the command ends
    SmartDashboard.putBoolean("ArmToPositionFinished", isFinished);
  }
  
}
