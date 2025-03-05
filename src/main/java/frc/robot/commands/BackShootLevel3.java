package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class BackShootLevel3 extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;

  public BackShootLevel3(Arm m_arm, Wrist m_wrist) {

    m_wristsub = m_wrist;
    m_armsub = m_arm;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new BackLevel3Reach(m_armsub),
    new WaitCommand(0.15),
    new InstantCommand(() -> m_wristsub.setWheelMotor(9)),
    new WaitCommand(0.2),
    new InstantCommand(() -> m_wristsub.setWheelMotor(0))
    );

  }
}
