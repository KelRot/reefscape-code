package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Wrist.*;

public class IntakeCmd extends Command {
    private final Arm m_arm;
    private final Wrist m_wrist;
    private final Timer m_timer;
    private boolean timeron, m_finished;
    public IntakeCmd(Arm arm, Wrist wrist) {
        m_timer = new Timer();
        m_arm = arm;
        m_wrist = wrist;
        timeron = false;
        m_finished = false;
        addRequirements(arm, wrist);
    }

    @Override
    public void execute() {
        m_arm.setSetPoint(Constants.LevelAngles.DefaultAngle);
        m_wrist.setSetPoint(Constants.LevelAngles.DefaultAngleWrist);
        if(m_wrist.getSensor()) {
            m_finished = false;
            m_wrist.setWheelMotor(4);
        } else { 
            if(!timeron) {
            m_timer.reset();
            m_timer.start();
            timeron=true;
        } else if (m_timer.get() > 0.07) {
            m_wrist.setWheelMotor(0);
            m_finished = true;
            }
        }

    }
 
    @Override
    public void end(boolean isFinished) {
        m_wrist.setWheelMotor(0);
        timeron = false;
        m_finished = false;
    }
    @Override
    public boolean isFinished() {
      return m_finished;
    }
}