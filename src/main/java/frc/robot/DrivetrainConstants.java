// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DrivetrainConstants {
    public static final double maxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
    public static final double trackWidthMeters = Units.inchesToMeters(18.78);
    public static final double rotationDiameter = trackWidthMeters * Math.PI * Math.sqrt(2);
    public static final double rotationsPerSecond = maxSpeedMetersPerSecond / rotationDiameter;
    public static final double maxAngularVelocityRadiansPerSecond = 2 * Math.PI * rotationsPerSecond;
    public static Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final double currentLimit = 60;
    public static final double autoMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond * 0.8;
    public static final double estimatedKp = 12/(maxSpeedMetersPerSecond/ (Units.inchesToMeters(TunerConstants.kWheelRadiusInches) * 2 * Math.PI));

}
