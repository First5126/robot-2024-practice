package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(DrivetrainConstants.maxSpeedMetersPerSecond * 0.05)
            .withRotationalDeadband(DrivetrainConstants.maxAngularVelocityRadiansPerSecond * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command gasPedalCommand(Supplier<Double> throttleSupplier, Supplier<Double> rotationSupplier,
            Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
        return run(() -> {
            double throttle = throttleSupplier.get();
            double rotation = rotationSupplier.get();
            double x = xSupplier.get();
            double y = ySupplier.get();
            double angle = Math.atan2(x, y) + Math.PI / 2;
            x = Math.cos(angle) * throttle;
            y = Math.sin(angle) * throttle;
            setControl(m_drive
                    .withVelocityX(-percentOutputToMetersPerSecond(x))
                    .withVelocityY(percentOutputToMetersPerSecond(y))
                    .withRotationalRate(-percentOutputToRadiansPerSecond(rotation)));
        });
    }

    public static double percentOutputToMetersPerSecond(double percentOutput) {
        return DrivetrainConstants.maxSpeedMetersPerSecond * percentOutput;
    }

    public static double percentOutputToRadiansPerSecond(double percentOutput) {
        return DrivetrainConstants.maxAngularVelocityRadiansPerSecond * percentOutput;
    }
}
