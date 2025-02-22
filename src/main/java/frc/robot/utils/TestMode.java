package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm.ArmConstants;

public class TestMode {

    public TestMode() {
        SmartDashboard.putBoolean("testMode", false);
    }

    public void ArmPIDConstants() {
        boolean isOn = SmartDashboard.getBoolean("isRioPIDController", false);
        if (isOn) {
            SmartDashboard.putNumber("armKp", ArmConstants.rioKP);
            SmartDashboard.putNumber("armKi", ArmConstants.rioKI);
            SmartDashboard.putNumber("armKd", ArmConstants.rioKD);
            SmartDashboard.clearPersistent("armKPspark");
            SmartDashboard.clearPersistent("armKIsparkarmKIspark");
            SmartDashboard.clearPersistent("armKDspark");
        } else {
            SmartDashboard.putNumber("armKPspark", ArmConstants.sparkKP);
            SmartDashboard.putNumber("armKIspark", ArmConstants.sparkKI);
            SmartDashboard.putNumber("armKDspark", ArmConstants.sparkKD);
            SmartDashboard.clearPersistent("armKp");
            SmartDashboard.clearPersistent("armKi");
            SmartDashboard.clearPersistent("armKd");

        }
    }

    public void WristPIDConstants() {
        SmartDashboard.putNumber("wristKPspark", ArmConstants.sparkKP);
        SmartDashboard.putNumber("wristKIspark", ArmConstants.sparkKI);
        SmartDashboard.putNumber("wristKDspark", ArmConstants.sparkKD);
    }

    public void clearConstants() {
        SmartDashboard.clearPersistent("armKPspark");
        SmartDashboard.clearPersistent("armKIsparkarmKIspark");
        SmartDashboard.clearPersistent("armKDspark");
        SmartDashboard.clearPersistent("armKp");
        SmartDashboard.clearPersistent("armKi");
        SmartDashboard.clearPersistent("armKd");
        SmartDashboard.clearPersistent("wristKPspark");
        SmartDashboard.clearPersistent("wristKIsparkarmKIspark");
        SmartDashboard.clearPersistent("wristKDspark");
    }
    public void periodic() {
        boolean testMode = SmartDashboard.getBoolean("testMode", false);
        if (testMode) {
            ArmPIDConstants();
            WristPIDConstants();
        } else {
            clearConstants();
        }
    }
}
