package com.chsrobotics.ftccore.engine.navigation.path;

import com.chsrobotics.ftccore.geometry.Position;

import org.apache.commons.math3.geometry.Point;

public abstract class MotionProfile {
    public double maxAccel;
    public double maxVelocity;
    public double distance;

    public MotionProfile(double accel, double velocity)
    {
        maxAccel = accel;
        maxVelocity = velocity;
    }

    public void calculateProfile(Position p0, Position p1)
    {
        double ac = Math.abs(p1.y - p0.y);
        double cb = Math.abs(p1.x - p0.x);

        distance = Math.hypot(ac, cb);
    }

    public abstract double getOutput(double time);

    public abstract double getSplineOutput(double time, double t);

    public void reset() {

    }

}
