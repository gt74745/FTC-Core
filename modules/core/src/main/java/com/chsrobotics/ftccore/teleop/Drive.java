package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

public class Drive {
    public final HardwareManager manager;
    private long prevTime = System.currentTimeMillis();
    private final UserDriveLoop loop;
    private final Builder.ScaleMode mode;
    private final PID rotationController = new PID(new PIDCoefficients(0.2, 0, 0));

    public Drive (HardwareManager manager, UserDriveLoop loop, Builder.ScaleMode mode)
    {
        this.mode = mode;
        this.manager = manager;
        this.loop = loop;
    }

    private void driveLoop()
    {
        boolean isRotationControllerActive = true;
        double lastTheta = 0;
        long rotationControllerTimeoutStart = 0;
        while (!manager.opMode.isStopRequested())
        {
            loop.loop();

            prevTime = System.currentTimeMillis();

            Gamepad gamepad1 = manager.opMode.gamepad1;

            double joystick_y;
            double joystick_x;
            double joystick_power = 0;
            double negative_power;
            double positive_power;
            double theta;
            double orientation;
            double rot_power;
            Orientation gyro_angles;

            joystick_y = -gamepad1.left_stick_y;
            joystick_x = gamepad1.left_stick_x == 0 ? 0.001 : gamepad1.left_stick_x;

            rot_power = (manager.thetaReversed ? -1 : 1) * gamepad1.right_stick_x;

            gyro_angles = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            theta = gyro_angles.firstAngle - manager.IMUReset - manager.offset;

            theta *= manager.thetaReversed ? -1 : 1;

            // todo: override for rotation pid
            // todo: dpad for arm heights
            isRotationControllerActive = System.currentTimeMillis() - rotationControllerTimeoutStart > 250;
            if (rot_power == 0 && isRotationControllerActive) {
                lastTheta = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - manager.offset;
                if (lastTheta > 2 * Math.PI) {
                    lastTheta -= 2 * Math.PI;
                } else if (lastTheta < 0) {
                    lastTheta += 2 * Math.PI;
                }
                double thetaError = lastTheta - theta;

                boolean isCounterClockwise = false;

                if (Math.abs(lastTheta - (theta - (2 * Math.PI))) < Math.abs(thetaError))
                {
                    thetaError = lastTheta - (theta - (2 * Math.PI));
                    isCounterClockwise = true;
                }

                if (Math.abs(lastTheta - (theta + (2 * Math.PI))) < thetaError)
                {
                    thetaError = lastTheta - (theta + (2 * Math.PI));
                    isCounterClockwise = true;
                }

                if (thetaError > 0 && (thetaError < Math.PI))
                    isCounterClockwise = true;

                if (thetaError < 0 && (thetaError > -Math.PI))
                    isCounterClockwise = false;

                rot_power = (isCounterClockwise ? -1 : 1) * rotationController.getOutput(Math.abs(thetaError), 0);
            } else {
                rotationControllerTimeoutStart = System.currentTimeMillis();
            }

            double rawPower = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

            switch (mode)
            {
                case SIN_SCALE:
                    joystick_power = (Math.sin((rawPower * (Math.PI/2)) - (Math.PI / 2d))) + 1;
                    break;
                case QUADRATIC_SCALE:
                    joystick_power = Math.pow(rawPower, 2);
                    break;
                default:
                    joystick_power = rawPower;
                    break;
            }

            if (gamepad1.left_bumper) {
                joystick_x = Math.round(joystick_x);
                joystick_y = Math.round(joystick_y);
            }

            orientation = (Math.atan2(joystick_y, joystick_x) - Math.PI / 4) - theta;

            negative_power = (joystick_power * Math.sin(orientation));
            positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                    negative_power;

            move(positive_power, negative_power, rot_power);
        }
    }

    public void move(double posinput, double neginput, double rotinput)
    {
//        manager.opMode.telemetry.addData("lf", -posinput - rotinput);
//        manager.opMode.telemetry.addData("rf", neginput - rotinput);
//        manager.opMode.telemetry.addData("lb", -neginput - rotinput);
//        manager.opMode.telemetry.addData("rb", posinput - rotinput);
        manager.getLeftFrontMotor().setPower((manager.linearSpeed * -posinput) - (manager.rotSpeed * rotinput));
        manager.getRightFrontMotor().setPower((manager.linearSpeed * neginput) - (manager.rotSpeed * rotinput));
        manager.getLeftBackMotor().setPower((manager.linearSpeed * -neginput) - (manager.rotSpeed * rotinput));
        manager.getRightBackMotor().setPower((manager.linearSpeed * posinput) - (manager.rotSpeed * rotinput));
    }

    public void runDriveLoop() {
        driveLoop();
    }

    public static class Builder
    {
        private final HardwareManager manager;
        private UserDriveLoop loop;
        private ScaleMode mode = ScaleMode.LINEAR;

        public Builder(HardwareManager manager)
        {

            this.manager = manager;
        }

        public Builder addUserLoop(UserDriveLoop loop)
        {
            this.loop = loop;
            return this;
        }

        public Builder useScale(ScaleMode mode)
        {
            this.mode = mode;
            return this;
        }

        public enum ScaleMode
        {
            LINEAR,
            SIN_SCALE,
            QUADRATIC_SCALE
        }

        public Drive build()
        {
            return new Drive(manager, loop, mode);
        }
    }
}
