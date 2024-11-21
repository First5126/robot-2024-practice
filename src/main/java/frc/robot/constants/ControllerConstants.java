// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ControllerConstants {
    public static final double DEADBAND = 0.05;

    public static double modifyAxis(double value) {
		return modifyAxis(value, 1);
	}
	public static double modifyAxis(double value, int exponent) {
		// Deadband
		value = MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);
		value = Math.copySign(Math.pow(value, exponent), value);
		return value;
	}
}
