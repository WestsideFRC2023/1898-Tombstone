package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;
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
  public ArmToPosition(Arm arm, double basePos, double wristPos) {
    mArm = arm;

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
  public ArmToPosition(Arm arm, double basePos, double wristPos, double delayTime) {
    mArm = arm;

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

    if (mArm.baseAtTarget()) {
        mArm.passTargets();
    } else if (mArm.wristStowed()) {
        mArm.setTargets(baseTarget, ArmConstants.WRIST_STOWED_POS);
        mArm.passTargets();
    } else {
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
  public boolean isFinished() {
    return timer.hasElapsed(delay);
  }

  @Override
  public void end(boolean isFinished) {
    // System.out.println("ArmToPosition Finished");
  }
  
}
