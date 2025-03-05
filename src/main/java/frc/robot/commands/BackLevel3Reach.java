package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class BackLevel3Reach extends Command {
  private final Arm m_armsub;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BackLevel3Reach(Arm m_arm) {
    m_armsub = m_arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armsub.reachSetPoint(Constants.LevelAngles.BackLevel3);   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armsub.reachSetPoint(Constants.LevelAngles.DefaultAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
