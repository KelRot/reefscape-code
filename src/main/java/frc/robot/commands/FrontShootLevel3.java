package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class FrontShootLevel3 extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;

  public FrontShootLevel3(Arm m_arm, Wrist m_wrist) {

    m_wristsub = m_wrist;
    m_armsub = m_arm;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.FrontLevel3)),
    new WaitCommand(0.25),
    new InstantCommand(() -> m_wristsub.setWheelMotor(9)),
    new WaitCommand(0.6),
    new InstantCommand(() -> m_wristsub.setWheelMotor(0)),
    new InstantCommand(() -> m_armsub.setSetPoint(Constants.LevelAngles.DefaultAngle))
    );

  }
}
