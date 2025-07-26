package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.RobotStatusManager.RobotStatus;
import frc.robot.subsystems.Wrist.*;

public class ShootLevel2 extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;
  private final RobotStatusManager robotStatusManager;
  

  public ShootLevel2(Arm m_arm, Wrist m_wrist, RobotStatusManager m_robotStatusManager) {

    m_wristsub = m_wrist;
    m_armsub = m_arm;
    robotStatusManager = m_robotStatusManager;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new InstantCommand(() -> robotStatusManager.setStatus(RobotStatus.ShootingL2)),
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.Level2)),
    new WaitCommand(0.4),
    new InstantCommand(() -> m_wristsub.setWheelMotor(9)),
    new WaitCommand(0.7),
    new InstantCommand(() -> m_wristsub.setWheelMotor(0)),
    new InstantCommand(() -> robotStatusManager.setStatus(RobotStatus.Nothing)),
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.DefaultAngle))
    );

  }
}
