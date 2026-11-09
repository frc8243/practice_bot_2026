package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(10); // CAN ID
    private boolean isGreen = false; // Elastic shows this

    public LEDSubsystem() {
        var cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 0.4;
        cfg.LED.StripType = StripTypeValue.GRB;
        candle.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LED/GREEN", isGreen);
    }

    // Indices 0-7 are the CANdle's onboard LEDs, 8-399 the attached strip.
    // 399 is the highest index the CANdle supports; anything past it is ignored.
    private static final int kAll = 128;

    public void setGreen() {
        apply(new RGBWColor(0, 255, 0, 0));
        isGreen = true;
    }

    public void turnOff() {
        apply(new RGBWColor(0, 0, 0, 0));
        isGreen = false;
    }

    public void setBlue() {
        apply(new RGBWColor(0, 0, 255, 0));
        isGreen = true;
    }

    public void setPurple() {
        apply(new RGBWColor(255, 0, 255, 0));
        isGreen = true;
    }

    private void apply(RGBWColor color) {
        var status = candle.setControl(new SolidColor(0, kAll).withColor(color));
        SmartDashboard.putString("LED/Status", status.toString());
    }
}
