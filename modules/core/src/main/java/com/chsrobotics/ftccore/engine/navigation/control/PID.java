package com.chsrobotics.ftccore.engine.navigation.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.chsrobotics.ftccore.geometry.Position;


public class PID {
    private final double kP;
    public final double kI;
    private final double kD;
    public double errorSum;
    private double maxErrorSum;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
        maxErrorSum = 100 / kI;
    }

    public double getOutput(double error, double speed) {
        errorSum += error;
        errorSum = Math.min(errorSum, maxErrorSum);
        return (kP * error) + (kI * errorSum) - (kD * speed);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }

    public void resetSum() {
        errorSum = 0;
    }
}
