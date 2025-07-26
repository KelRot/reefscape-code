package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class BackAlgeaRemover extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;

  public BackAlgeaRemover(Arm m_arm, Wrist m_wrist) {

    m_wristsub = m_wrist;
    m_armsub = m_arm;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.BackAlgaeRemover)),
    new InstantCommand(() -> m_wristsub.setSetPoint(Constants.LevelAngles.BackAlgaeRemoverWrist))
    );

  }
}
