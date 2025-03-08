package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.RobotStatusManager.RobotStatus;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class BackShootLevel3 extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;
  private final RobotStatusManager robotStatusManager;

  public BackShootLevel3(Arm m_arm, Wrist m_wrist, RobotStatusManager m_robotStatusManager) {

    m_wristsub = m_wrist;
    m_armsub = m_arm; 
    robotStatusManager = m_robotStatusManager;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new InstantCommand(() -> robotStatusManager.setStatus(RobotStatus.ShootingL3)),
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.BackLevel3)),
    new WaitCommand(0.2),
    new InstantCommand(() -> m_wristsub.setSetPoint(Constants.LevelAngles.BackLevel3Wrist)),
    new WaitCommand(1.3),
    new InstantCommand(() -> m_wristsub.setWheelMotor(-11)),
    new WaitCommand(1),
    new InstantCommand(() -> m_wristsub.setSetPoint(Constants.LevelAngles.DefaultAngleWrist - 0.1)),
    new InstantCommand(() -> m_wristsub.setWheelMotor(0)),
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.DefaultAngle)),
     new WaitCommand(0.5),
    new InstantCommand(() -> m_wristsub.setSetPoint(Constants.LevelAngles.DefaultAngleWrist)),
    new InstantCommand(() -> robotStatusManager.setStatus(RobotStatus.Nothing))
    );

  }
}
