package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class PivotArm extends SubsystemBase {
    // numbers to measure (guesses OK in sim)
    private static final int kLeaderId = 8;

    private static final double kGearRatio = 12.0;
    private static final double kMinAngle = Math.toRadians(-135);
    private static final double kMaxAngle = Math.toRadians(0);
    private static final double kStartAngle = kMinAngle;
    // fields
    private final SparkMax leader = new SparkMax(kLeaderId, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();

    public PivotArm() {
        double factor = 2 * Math.PI / kGearRatio; // motor turns -> arm radians
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        leaderConfig.encoder.positionConversionFactor(factor).velocityConversionFactor(factor / 60);
        leaderConfig
                .softLimit
                .forwardSoftLimit(kMaxAngle)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(kMinAngle)
                .reverseSoftLimitEnabled(true);
        leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder.setPosition(kStartAngle);
    }

    public Command manual(DoubleSupplier stick) {
        return run(() -> leader.set(stick.getAsDouble() * 0.25));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Angle deg", Units.radiansToDegrees(encoder.getPosition()));
    }
}
