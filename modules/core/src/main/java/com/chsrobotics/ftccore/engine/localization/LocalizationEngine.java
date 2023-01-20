package com.chsrobotics.ftccore.engine.localization;


import com.chsrobotics.ftccore.engine.localization.localizer.EncoderLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.IMULocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.Localizer;
import com.chsrobotics.ftccore.engine.localization.localizer.OdometryLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.VisionLocalizer;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;

public class LocalizationEngine {
    /*
    The localization engine can be used by actions or anywhere else across the system. It is responsible for fusing the
    inputs of the different localizers using a Kalman filter into a more accurate output.
    */

    public Position currentPosition = new Position(0, 0, 0);
    public Position lastPosition = new Position(0, 0, 0);

    private List<Localizer> localizers;

    private final HardwareManager manager;

    public LocalizationEngine(HardwareManager manager)
    {
        this.manager = manager;
        initializeLocalization();
    }

    /**
     * Computes position of robot through a Kalman filter.
     * @return Current position of the robot in X, Y, and T (radians)
     */
    public Position getCurrentPosition()
    {
        //Kalman filter
        List<Position> positions = new ArrayList<>();

        for (Localizer localizer : localizers) {
            positions.add(localizer.getRobotPosition(lastPosition));
        }

        localizers.get(0).updateRobotPosition(positions.get(0));

        lastPosition = currentPosition;
        currentPosition = positions.get(0);
        return currentPosition; //Temporarily returning only encoder based position.
    }

    private void initializeLocalization()
    {
        localizers = new ArrayList<>();

        if (manager.accessoryOdometryPods.length == 2) {
            localizers.add(new OdometryLocalizer(null, manager));
        }

        if (manager.driveMotors.length == 4) {
            localizers.add(new EncoderLocalizer(null, manager));
        }

        if (manager.isImuLocalEnabled()) {
            localizers.add(new IMULocalizer(null, manager));
        }

        if (manager.accessoryCameras.length > 0) {
            localizers.add(new VisionLocalizer(null, manager));
        }
    }
}
