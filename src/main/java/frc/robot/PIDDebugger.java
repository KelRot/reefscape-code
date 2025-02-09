package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDDebugger {

    public PIDDebugger() {

    }

    public void setSparkPIDControllerToDashboard(String prefix, double kP, double kI, double kD, double kMinOutput,
            double kIz, double kMaxOutput) {
        SmartDashboard.putNumber(prefix + "P", kP);
        SmartDashboard.putNumber(prefix + "I", kI);
        SmartDashboard.putNumber(prefix + "D", kD);
        SmartDashboard.putNumber(prefix + "MinOutput", kMinOutput);
        SmartDashboard.putNumber(prefix + "MaxOutput", kMaxOutput);
        SmartDashboard.putNumber(prefix + "Iz", kIz);
    }

}