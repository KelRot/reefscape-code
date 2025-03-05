package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Optional;



import frc.robot.Constants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootLevel2 extends SequentialCommandGroup {

  private final Arm m_armsub;
  private final Wrist m_wristsub;

  public ShootLevel2(Arm m_arm, Wrist m_wrist) {

    m_wristsub = m_wrist;
    m_armsub = m_arm;
    addRequirements(m_arm, m_wrist);

    addCommands(
    new InstantCommand(() -> m_arm.reachSetPoint(Constants.LevelAngles.Level2)),
    new WaitCommand(0.15),
    new InstantCommand(() -> m_wristsub.setWheelMotor(6)),
    new WaitCommand(0.2),
    new InstantCommand(() -> m_wristsub.setWheelMotor(0)),
    new InstantCommand(() -> m_arm.reachSetPoint(Constants.LevelAngles.DefaultAngle))
    );

  }
}
