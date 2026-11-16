package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    public static final double kStowDeg = -136;
    public static final double kMidDeg = -84;
    public static final double kDeploy = 5;
    // Tuning numbers
    private static final double kP = 0.05;
    private static final double kG = -0.5;
    private static final double kMaxVolts = 3.0;

    private final PIDController pid = new PIDController(kP, 0, 0.0);
    private double targetDeg = kStowDeg;

    private double gravityVolts() {
        return kG * Math.cos(encoder.getPosition());
    }

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

    private double angleDeg() {
        return Units.radiansToDegrees(encoder.getPosition());
    }

    public Command goToAngle(double degrees) {
        return run(
                () -> {
                    targetDeg = degrees;
                    double push = pid.calculate(angleDeg(), degrees);
                    push = MathUtil.clamp(push, -kMaxVolts, kMaxVolts);
                    leader.setVoltage(push + gravityVolts());
                });
    }

    public Command manual(DoubleSupplier stick) {
        return run(() -> leader.set(stick.getAsDouble() * 0.25));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Angle deg", Units.radiansToDegrees(encoder.getPosition()));
        SmartDashboard.putNumber("Arm/Target deg", targetDeg);
    }
}
