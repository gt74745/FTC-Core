package com.chsrobotics.ftccore.engine.navigation;

import android.os.Environment;

import com.chsrobotics.ftccore.Log;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.path.MotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.engine.navigation.control.*;
import com.chsrobotics.ftccore.pipeline.Pipeline;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class NavigationEngine {
    private final HardwareManager hardware;
    public final LocalizationEngine localization;
    public final PID linearController;
    public final PID rotationController;
    public Position position = new Position();
    double magnitude;
    private double error;
    private double lastError;
    private long time;
    private long lastTime;
    private double thetaError;
    private boolean isCounterClockwise;
    private Position lastPosition = new Position(0, 0, 0);

    public NavigationEngine(LocalizationEngine localization, HardwareManager hardware)
    {
        this.hardware = hardware;
        this.localization = localization;
        this.linearController = hardware.linearCtrler;
        this.rotationController = hardware.rotCtrler;
    }

    public boolean isTargetReached(Position destination)
    {
        error = Math.sqrt(Math.pow(destination.y - position.y, 2) + Math.pow(destination.x - position.x, 2));

        if (destination.time > 0.0 && Pipeline.time.time(TimeUnit.MILLISECONDS) / 1000d > destination.time)
        {
            thetaError = destination.t - position.t;
        } else if (destination.time == 0.0)
            thetaError = destination.t - position.t;
        else
            thetaError = 0;

        isCounterClockwise = false;

        if (Math.abs(destination.t - (position.t - (2 * Math.PI))) < Math.abs(thetaError))
        {
            thetaError = destination.t - (position.t - (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (Math.abs(destination.t - (position.t + (2 * Math.PI))) < thetaError)
        {
            thetaError = destination.t - (position.t + (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (thetaError > 0 && (thetaError < Math.PI))
            isCounterClockwise = true;

        if (thetaError < 0 && (thetaError > -Math.PI))
            isCounterClockwise = false;

        return (error < hardware.tolerances.linear && Math.abs(thetaError) < hardware.tolerances.rotational);
    }

    public void navigateInALinearFashion(Position destination, MotionProfile profile)
    {
        if (profile != null)
        {
            Position pos = localization.currentPosition;
            double posError = Math.sqrt(Math.pow(destination.y - pos.y, 2) + Math.pow(destination.x - pos.x, 2));

            if (posError > (profile.distance / 2))
            {
                error = profile.getOutput(Pipeline.time.time(TimeUnit.MILLISECONDS) / 1000d);
            } else if (posError > (profile.distance - 100))
                error = profile.distance - profile.getOutput(Pipeline.time.time(TimeUnit.MILLISECONDS) / 1000d);
            else
                error = posError;
        }

        position = localization.getCurrentPosition();

        double orientation = Math.atan2(destination.y - position.y, destination.x - position.x) - Math.PI / 4 - position.t;

        time = System.currentTimeMillis();

        if (profile != null && hardware.profileCtrler != null)
            magnitude = hardware.profileCtrler.getOutput(error, (error - lastError) / (time - lastTime));

        magnitude = linearController.getOutput(error, (error - lastError) / (time - lastTime));

        double negOutput = magnitude * Math.sin(orientation);

        double posOutput = magnitude * Math.cos(orientation);
        if (orientation == 0) {
            posOutput = negOutput;
        }

        double thetaOutput = Math.abs(thetaError) >= hardware.tolerances.rotational ? rotationController.getOutput(Math.abs(thetaError), 0) : 0;

        lastTime = time;
        lastError = error;

        if (hardware.debugMode) {
            hardware.opMode.telemetry.addData("error", error);
			hardware.opMode.telemetry.addData("direction", orientation);
            hardware.opMode.telemetry.update();
        }

        hardware.getLeftFrontMotor().setVelocity((-posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getRightFrontMotor().setVelocity((negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getLeftBackMotor().setVelocity((-negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
        hardware.getRightBackMotor().setVelocity((posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
    }

    public void navigateInANonLinearFashion(Position dest, ParametricSpline spline)
    {


        position = localization.getCurrentPosition();
        if (lastPosition.x == 0 && lastPosition.y == 0 && lastPosition.t == 0) {
            lastPosition = position;
        }

        double orientation, negOutput, posOutput, magnitude;
//        while (!hardware.opMode.isStopRequested())
//        {
//            hardware.opMode.telemetry.addData("Short by (absolute)", Math.sqrt(Math.pow(positions.get(positions.size() - 1).x - position.x, 2) + Math.sqrt(Math.pow(positions.get(positions.size() - 1).y - position.y, 2))));
//            hardware.opMode.telemetry.addData("Short by (ratio)", Math.sqrt(Math.pow(positions.get(positions.size() - 1).x - position.x, 2) + Math.sqrt(Math.pow(positions.get(positions.size() - 1).y - position.y, 2))) / spline.splineDistance);
//            hardware.opMode.telemetry.update();
//        }

        Pipeline.distTraveled += Math.sqrt(Math.pow(position.x - lastPosition.x, 2) + Math.pow(position.y - lastPosition.y, 2));

        Pipeline.t = Pipeline.distTraveled / spline.splineDistance;

        if (Pipeline.t > 1) {
            return;
        }

        if (spline.xSpline.derivative().value(Pipeline.t) > 0)
            orientation = Math.atan(spline.getDerivative(Pipeline.t)) - (Math.PI / 4);
        else
            orientation = Math.atan(spline.getDerivative(Pipeline.t))  + Math.PI- Math.PI / 4;
        negOutput = 1500 * Math.sin(orientation);

        if (orientation == 0) {
            posOutput = negOutput;
        } else {
            posOutput = 1500 * Math.cos(orientation);
        }

        double thetaOutput = rotationController.getOutput(Math.abs(thetaError), 0);

        lastPosition = position;

        hardware.opMode.telemetry.addData("t", Pipeline.t);
        hardware.opMode.telemetry.addData("dist traveled", Pipeline.distTraveled);
        hardware.opMode.telemetry.addData("pos", posOutput);
        hardware.opMode.telemetry.addData("neg", negOutput);
        hardware.opMode.telemetry.addData("X", position.x);
        hardware.opMode.telemetry.addData("Y", position.y);
        hardware.opMode.telemetry.update();

        hardware.getLeftFrontMotor().setVelocity((-posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        hardware.getRightFrontMotor().setVelocity(( negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        hardware.getLeftBackMotor().setVelocity((-negOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));
        hardware.getRightBackMotor().setVelocity((posOutput) - ((isCounterClockwise ? -1 : 1) * thetaOutput));


    }
}
