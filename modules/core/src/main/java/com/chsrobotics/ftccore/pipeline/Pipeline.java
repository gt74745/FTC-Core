package com.chsrobotics.ftccore.pipeline;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.actions.SetPrecisionAction;
import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.engine.navigation.NavigationEngine;
import com.chsrobotics.ftccore.engine.navigation.control.ParametricSpline;
import com.chsrobotics.ftccore.engine.navigation.control.SplineHelper;
import com.chsrobotics.ftccore.engine.navigation.path.MotionProfile;
import com.chsrobotics.ftccore.engine.navigation.path.Path;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class Pipeline {

    private HardwareManager manager;
    private ArrayList<PipelineStep> steps;
    private ArrayList<ContinuousAction> continuousActions;
    private final double conversion = Math.PI / 180;
    public final LocalizationEngine localization;

    public static ElapsedTime time = new ElapsedTime();
    private static Pipeline INSTANCE;
    public static double t = 0;
    public static double distTraveled = 0;


    public static Pipeline getInstance() {
        return INSTANCE;
    }

    private Pipeline(HardwareManager manager, ArrayList<PipelineStep> steps, ArrayList<ContinuousAction> continuousActions) {
        INSTANCE = this;
        this.manager = manager;
        this.steps = steps;
        this.continuousActions = continuousActions;
        this.localization = new LocalizationEngine(manager);
        t = 0;
        distTraveled = 0;

        time.startTime();
    }

    public void runContinuousActions() {
        for (ContinuousAction action : continuousActions) {
            action.execute();
        }
    }

    public void execute()
    {
        NavigationEngine navigationEngine = new NavigationEngine(localization, manager);

        t = 0;
        time.reset();
        navigationEngine.linearController.resetSum();
        navigationEngine.rotationController.resetSum();
        for (PipelineStep step : steps)
        {
            if (manager.opMode.isStopRequested())
            {
                break;
            }
            if (step.type == PipelineStep.StepType.NAVIGATION)
            {
                assert step.path != null;
                if (step.path.isCurved) {

                    double[] x = new double[step.path.positions.size()];
                    double[] y = new double[step.path.positions.size()];

                    for (int i = 0; i < step.path.positions.size(); i++) {
                        x[i] = step.path.positions.get(i).x;
                        y[i] = step.path.positions.get(i).y;
                    }

                    SplineHelper splineHelper = new SplineHelper();

                    ParametricSpline spline = splineHelper.computeSpline(x, y);
                    if (step.path.profile != null) {
                        step.path.profile.reset();
                    }
                    manager.setPrecisionMode(PrecisionMode.LOW);
                    for (int i = 0; i < step.path.positions.size(); i++) {
                        Position dest = step.path.positions.get(i);
                        if (manager.opMode.isStopRequested())
                        {
                            break;
                        }
                        while ((!navigationEngine.isTargetReached(dest) || i != step.path.positions.size() - 1) && !manager.opMode.isStopRequested() && t < 1) {
                            if (step.path.actions != null && step.path.actionTimes != null && !step.path.actions.isEmpty() && !step.path.actionTimes.isEmpty() && step.path.actionTimes.get(0) <= t) {
                                step.path.actions.get(0).execute();
                                step.path.actions.remove(0);
                                step.path.actionTimes.remove(0);
                            }
                            navigationEngine.navigateInANonLinearFashion(spline, step.path.profile, step.path.distanceCoefficient);
                            runContinuousActions();
                        }
                        navigationEngine.linearController.resetSum();
                        navigationEngine.rotationController.resetSum();
                    }
                    t = 0;
                    time.reset();
                    if (step.path.profile != null) {
                        step.path.profile.reset();
                    }
                    manager.setPrecisionMode(PrecisionMode.MEDIUM);
                    continue;
                }
                for (Position dest : step.path.positions) {
                    navigationEngine.linearController.resetSum();
                    navigationEngine.rotationController.resetSum();
                    if (step.path.profile != null && manager.profileCtrler != null)
                        manager.profileCtrler.resetSum();
                    if (manager.useDegrees) {
                        dest.t *= conversion;
                    }
                    if (manager.opMode.isStopRequested())
                    {
                        break;
                    }

                    if (step.path.profile != null)
                    {
                        step.path.profile.calculateProfile(localization.currentPosition, dest);
                    }

                    while (!navigationEngine.isTargetReached(dest) && !manager.opMode.isStopRequested() && (time.time() < dest.maxTime || dest.maxTime == 0)) {
                        navigationEngine.navigateInALinearFashion(dest, step.path.profile);
                        runContinuousActions();
                    }
                    time.reset();
                    navigationEngine.linearController.resetSum();
                    if (step.path.profile != null) {
                        step.path.profile.reset();
                    }
                }
            } else if (step.type == PipelineStep.StepType.ACTION)
            {
                assert step.action != null;
                step.action.execute();
            } else if (step.type == PipelineStep.StepType.STOP)
            {
                for (DcMotorEx motor : manager.driveMotors)
                    motor.setPower(0);
            }
        }
    }

    public static class Builder {

        private final HardwareManager manager;
        private final ArrayList<PipelineStep> steps;
        private final ArrayList<ContinuousAction> continuousActions;

        public Builder(HardwareManager manager) {
            this.manager = manager;
            steps = new ArrayList<>();
            continuousActions = new ArrayList<>();
        }

        public Builder addAction(Action action) {
            steps.add(new PipelineStep(action));
            return this;
        }

        public Builder addCurvedPath(MotionProfile profile, Position... positions) {
            steps.add(new PipelineStep(Path.curved(profile, positions)));
            return this;
        }

        public Builder addCurvedPath(MotionProfile profile, List<Action> actions, List<Double> actionTimes, Position... positions) {
            steps.add(new PipelineStep(Path.curved(profile, actions, actionTimes, positions)));
            return this;
        }

        public Builder addCurvedPath(MotionProfile profile, double distanceCoefficient, Position... positions) {
            steps.add(new PipelineStep(Path.curved(profile, distanceCoefficient, positions)));
            return this;
        }

        public Builder addCurvedPath(Position... positions) {
            steps.add(new PipelineStep(Path.curved(positions)));
            return this;
        }

        public Builder addLinearPath(boolean continuous, Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }

        public Builder addLinearPath(Position... positions) {
            steps.add(new PipelineStep(Path.linear(positions)));

            return this;
        }

        public Builder addLinearPath(MotionProfile profile, Position... positions) {
            steps.add(new PipelineStep(Path.linear(profile, positions)));

            return this;
        }

        public Builder addLinearPath(MotionProfile profile, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(Path.linear(profile, positions)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }
        public Builder addLinearPath(PrecisionMode precisionMode, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));
            return this;
        }
        public Builder addLinearPath(PrecisionMode precisionMode, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }

        public Builder addLinearPath(PrecisionMode precisionMode, MotionProfile profile, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(profile, positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));
            return this;
        }

        public Builder addLinearPath(PrecisionMode precisionMode, MotionProfile profile, boolean continuous, Position... positions) {
            steps.add(new PipelineStep(new SetPrecisionAction(manager, precisionMode)));
            steps.add(new PipelineStep(Path.linear(profile, positions)));
            steps.add(new PipelineStep(new SetPrecisionAction(manager, PrecisionMode.MEDIUM)));

            if (!continuous)
                steps.add(new PipelineStep(PipelineStep.StepType.STOP));

            return this;
        }

        public Builder addContinuousAction(ContinuousAction continuousAction)
        {
            continuousActions.add(continuousAction);
            return this;
        }

        public Pipeline build() {
            return new Pipeline(manager, steps, continuousActions);
        }
    }


}
