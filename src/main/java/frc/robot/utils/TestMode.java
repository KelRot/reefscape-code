package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Wrist.WristConstants;

public class TestMode {

    public TestMode() {
        SmartDashboard.putBoolean("testMode", true);
        SmartDashboard.putBoolean("isRioPIDController", true);
        boolean testMode = SmartDashboard.getBoolean("testMode", true);
        if (testMode) {
            ArmPIDConstants();
            WristPIDConstants();
            MotorDebugValues();
        }
        SmartDashboard.putBoolean("isRioPIDController", true);
    }

    public void ArmPIDConstants() {
        boolean isOn = SmartDashboard.getBoolean("isRioPIDController", true);
        if (isOn) {
            SmartDashboard.putNumber("Arm/RioKP", ArmConstants.rioKP);
            SmartDashboard.putNumber("Arm/RioKI", ArmConstants.rioKI);
            SmartDashboard.putNumber("Arm/RioKD", ArmConstants.rioKD);
        } else {
            SmartDashboard.putNumber("Arm/SparkKP", ArmConstants.sparkKP);
            SmartDashboard.putNumber("Arm/SparkKI", ArmConstants.sparkKI);
            SmartDashboard.putNumber("Arm/SparkKD", ArmConstants.sparkKD);

        }

        SmartDashboard.putNumber("Arm/TestAngle", 0);
    }

    public void WristPIDConstants() {
        SmartDashboard.putNumber("Wrist/RioKP", WristConstants.rioKP);
        SmartDashboard.putNumber("Wrist/RioKI", WristConstants.rioKI);
        SmartDashboard.putNumber("Wrist/RioKD", WristConstants.rioKD);
        SmartDashboard.putNumber("Wrist/SetPoint", -12);
    }

    public void MotorDebugValues() {
        SmartDashboard.putNumber("Climb/Opener Set", 0);
        SmartDashboard.putNumber("Climb/Closer Set", 0);
        SmartDashboard.putNumber("Wrist/Debug Voltage", 0);
        SmartDashboard.putNumber("Arm/Debug Voltage", 0);
    }

    
}
