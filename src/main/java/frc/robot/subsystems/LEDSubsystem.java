package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(50); // CAN ID
    private boolean isGreen = false; // Elastic shows this

    public LEDSubsystem() {}

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LED/GREEN", isGreen);
    }

    private static final int kAll = 127;

    public void setGreen() {
        candle.setControl(new SolidColor(0, kAll).withColor(new RGBWColor(0, 255, 0, 0)));
        isGreen = true;
    }

    public void turnOff() {
        candle.setControl(new SolidColor(0, kAll).withColor(new RGBWColor(0, 0, 0, 0)));
        isGreen = false;
    }

    public void setBlue() {
        candle.setControl(new SolidColor(0, kAll).withColor(new RGBWColor(0, 0, 255, 0)));
        isGreen = true;
    }
}
