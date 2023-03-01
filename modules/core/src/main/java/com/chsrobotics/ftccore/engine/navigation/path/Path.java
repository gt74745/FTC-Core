package com.chsrobotics.ftccore.engine.navigation.path;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.geometry.Position;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Path {

    public List<Position> positions;
    public boolean isCurved = false;

    public MotionProfile profile = null;
    public double distanceCoefficient = 1;
    public List<Action> actions = new ArrayList<>();
    public List<Double> actionTimes = new ArrayList<>();

    public Path(List<Position> positions) {
        this.positions = positions;
    }

    public Path(List<Position> positions, boolean isCurved, MotionProfile profile) {
        this.positions = positions;
        this.isCurved = isCurved;
        this.profile = profile;
    }

    public Path(List<Position> positions, boolean isCurved, MotionProfile profile, List<Action> actions, List<Double> actionTimes) {
        this.positions = positions;
        this.isCurved = isCurved;
        this.profile = profile;
        this.actions = actions;
        this.actionTimes = actionTimes;
    }

    public Path(List<Position> positions, boolean isCurved, MotionProfile profile, double distanceCoefficient) {
        this.positions = positions;
        this.isCurved = isCurved;
        this.profile = profile;
        this.distanceCoefficient = distanceCoefficient;
    }

    public static Path curved(MotionProfile profile, List<Action> actions, List<Double> actionTimes, Position... positions) {
        return new Path(Arrays.asList(positions), true, profile, actions, actionTimes);
    }

    public static Path curved(MotionProfile profile, Position... positions) {
        return new Path(Arrays.asList(positions), true, profile);
    }

    public static Path curved(MotionProfile profile, double distanceCoefficient, Position... positions) {
        return new Path(Arrays.asList(positions), true, profile, distanceCoefficient);
    }

    public static Path curved(Position... positions) {
        return new Path(Arrays.asList(positions), true, null);
    }

    public static Path linear(MotionProfile profile, Position... positions) {
        return new Path(Arrays.asList(positions), false, profile);
    }

    public static Path linear(Position... positions) {
        return new Path(Arrays.asList(positions), false, null);
    }

}
