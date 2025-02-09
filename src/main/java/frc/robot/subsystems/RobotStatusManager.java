package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDDebugger;
import frc.robot.Constants.GeneralConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class RobotStatusManager {
    public enum RobotStatus {
        Alignment(255,0,0, true), // Blinking Red
        CountDown_Intake(0, 255, 0, true),
        Has_Coral(0,255,0, false), // Blinking Green 
        ShootingL1(255, 0, 255, true),     // Blinking Purple
        ShootingL2(0, 0, 255, true),       // Blinking Blue
        ShootingL3(255, 0, 0, true),       // Blinking Red
        Removing_Algae(0, 255, 0, true),    // Blinking Green
        Climbing(0, 0, 0, false),  
        Test_Mode(0,0,0, false),        // Rainbow
        Nothing(0, 0, 0, false);           // Sliding 
        public final int r, g, b;
        public final boolean blinking;

        RobotStatus(int r, int g, int b, boolean blinking) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.blinking = blinking;
        }
    }
    private PIDDebugger m_PidDebugger;

    private RobotStatus currentStatus;
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final Timer blinkTimer;
    private int rainbowFirstPixelHue = 0;
    private int movingPixelIndex = 0;
    private int movingPixelHue = 0;
    private boolean isBlinkOn = true;
    private boolean ledSystemEnabled = true; // LED Sistemi Acık/Kapalı Durumu
    private static final int LED_LENGTH = GeneralConstants.LED_LENGTH;

    public RobotStatusManager() {
        m_PidDebugger = new PIDDebugger();
        this.currentStatus = RobotStatus.Nothing;

        // LED Şeridi Baslat
        ledStrip = new AddressableLED(GeneralConstants.LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        ledStrip.setLength(LED_LENGTH);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        // Timer Baslat
        blinkTimer = new Timer();
        blinkTimer.start();
        
        SmartDashboard.putBoolean("LED System", ledSystemEnabled);
    }

    public void setStatus(RobotStatus newStatus) {
        if (this.currentStatus != newStatus) { // Aynı duruma tekrar geçme
            this.currentStatus = newStatus;
            SmartDashboard.putString("Robot Status", newStatus.name());
        }
    }

    public RobotStatus getStatus() {
        return this.currentStatus;
    }

    public void toggleLEDSystem() {
        ledSystemEnabled = !ledSystemEnabled;
        SmartDashboard.putBoolean("LED System", ledSystemEnabled);

        if (!ledSystemEnabled) {
            turnOffLEDs();
        }
    }

    private void turnOffLEDs() {
        for (int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        ledStrip.setData(ledBuffer);
    }

    private void updateLEDBuffer(int r, int g, int b) {
        for (int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
    }

    private void updateRainbowEffect() {
        for (int i = 0; i < LED_LENGTH; i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / LED_LENGTH)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3; // Renk kaydırma efekti
        rainbowFirstPixelHue %= 180;
        ledStrip.setData(ledBuffer);
    }

    private void updateMovingPixelEffect() {
        // Tüm LED'leri sıfırla
        for (int i = 0; i < LED_LENGTH; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        // Hareket eden pikseli ayarla
        ledBuffer.setHSV(movingPixelIndex, movingPixelHue, 255, 255);

        // Pikseli kaydır
        movingPixelIndex++;
        if (movingPixelIndex >= LED_LENGTH) {
            movingPixelIndex = 0;
            movingPixelHue = (movingPixelHue + 45) % 180; // Renk değistir
        }
        ledStrip.setData(ledBuffer);
    }

    public void periodic() {
        if (!ledSystemEnabled) {
            return; // LED Sistemi Kapalıysa Çalışma
        }

        if (currentStatus == RobotStatus.Climbing) {
            updateRainbowEffect();
            return;
        }

        if (currentStatus == RobotStatus.Nothing) {
            updateMovingPixelEffect();
            return;
        }
        if (currentStatus == RobotStatus.Test_Mode) {
            m_PidDebugger.setSparkPIDControllerToDashboard("ankle", 0,0,0,0,0,0);
            return;
        }

        if (currentStatus.blinking && blinkTimer.get() > 0.3) {
            isBlinkOn = !isBlinkOn;
            blinkTimer.reset();

            if (isBlinkOn) {
                updateLEDBuffer(currentStatus.r, currentStatus.g, currentStatus.b);
            } else {
                updateLEDBuffer(0, 0, 0);
            }
        }
    }
}
