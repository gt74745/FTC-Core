package com.chsrobotics.ftccore.geometry;

import com.chsrobotics.ftccore.actions.Action;

public class Position {
    public double x;
    public double y;
    public double t;
    public double time = 0.0;
    public double maxTime = 0.0;

    public Position() {

    }

    public Position(double x, double y, double t)
    {
        this.x = x;
        this.y = y;
        this.t = t;
    }

    public Position(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Position(double x, double y, double t, double secondsBeforeRotation)
    {
        this.x = x;
        this.y = y;
        this.t = t;
        this.time = secondsBeforeRotation;
    }

    public Position(double x, double y, double t, double secondsBeforeRotation, double maxTime)
    {
        this.x = x;
        this.y = y;
        this.t = t;
        this.time = secondsBeforeRotation;
        this.maxTime = maxTime;
    }
}
