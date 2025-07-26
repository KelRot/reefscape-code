package frc.robot.commands;

import frc.robot.subsystems.Drive.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignCommand extends Command {
  private final Swerve m_drive;
  private final AprilTagAligner m_aligner;
  private final double xSpeed, ySpeed, rotationSpeed;
  private boolean m_finished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignCommand(Swerve swerve, AprilTagAligner align) {
    m_drive = swerve;
    m_aligner = align;
    xSpeed = m_aligner.getXSpeed();
    ySpeed = m_aligner.getYSpeed();
    rotationSpeed = m_aligner.getrotationSpeed();

    addRequirements(swerve, align);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_finished) {
    m_drive.driveFieldOriented(new ChassisSpeeds(xSpeed, -ySpeed, rotationSpeed));
    } else {
      m_finished = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean m_finished) {
    m_finished = false;
    m_drive.driveFieldOriented(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
