package org.firstinspires.ftc.teamcode.Units;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Chassis {

    private final double ALIGNMENT_ENCODER_DRIVE = 15.0;
    private final double ALIGNMENT_GYRO_TURN = 0.2;
    private final double ALIGNMENT_DISTANCE_DRIVE = 15.0;

    private static final double COUNTS_PER_MOTOR_REV = 560.0;
    private static final double WHEEL_DIAMETER_MM = 100.0;
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_MM * Math.PI);

    private DcMotor left_1;
    private DcMotor left_2;
    private DcMotor right_1;
    private DcMotor right_2;

    private ModernRoboticsI2cGyro gyro;
    private DistanceSensor range_1;
    private DistanceSensor range_2;
    private DistanceSensor range_3;

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private int holdAngle = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode opMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;

        left_1 = opMode.hardwareMap.get(DcMotor.class, "left_1");
        left_2 = opMode.hardwareMap.get(DcMotor.class, "left_2");
        right_1 = opMode.hardwareMap.get(DcMotor.class, "right_1");
        right_2 = opMode.hardwareMap.get(DcMotor.class, "right_2");
        gyro = opMode.hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        range_1 = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "range1");
        range_2 = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "range2");
        range_3 = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "range3");

        setDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opMode instanceof LinearOpMode)
            calibrateGyro();
    }

    private void calibrateGyro() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        gyro.calibrate();

        while (!linearOpMode.isStopRequested() && gyro.isCalibrating())
            linearOpMode.idle();

        gyro.resetZAxisIntegrator();
    }
    //------------------------------------------------------------------------//
    private void setDirection(DcMotorSimple.Direction... direction) {
        switch (direction.length) {
            case 4:
                left_1.setDirection(direction[0]);
                left_2.setDirection(direction[1]);
                right_1.setDirection(direction[2]);
                right_2.setDirection(direction[3]);
                break;
            case 2:
                left_1.setDirection(direction[0]);
                left_2.setDirection(direction[0]);
                right_1.setDirection(direction[1]);
                right_2.setDirection(direction[1]);
                break;
            default:
                left_1.setDirection(direction[0]);
                left_2.setDirection(direction[0]);
                right_1.setDirection(direction[0]);
                right_2.setDirection(direction[0]);
        }
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        left_1.setZeroPowerBehavior(behavior);
        left_2.setZeroPowerBehavior(behavior);
        right_1.setZeroPowerBehavior(behavior);
        right_2.setZeroPowerBehavior(behavior);
    }

    private void setMode(DcMotor.RunMode mode) {
        left_1.setMode(mode);
        left_2.setMode(mode);
        right_1.setMode(mode);
        right_2.setMode(mode);
    }

    public void setPower(double... power) {
        switch (power.length) {
            case 4:
                left_1.setPower(power[0]);
                left_2.setPower(power[1]);
                right_1.setPower(power[2]);
                right_2.setPower(power[3]);
                break;
            case 2:
                left_1.setPower(power[0]);
                left_2.setPower(power[0]);
                right_1.setPower(power[1]);
                right_2.setPower(power[1]);
                break;
            default:
                left_1.setPower(power[0]);
                left_2.setPower(power[0]);
                right_1.setPower(power[0]);
                right_2.setPower(power[0]);
        }
    }
    //------------------------------------------------------------------------//
    public void encoderDrive(double speed, int distance_mm, Direction direction, double timeout_seconds) {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        if (linearOpMode.opModeIsActive()) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            distance_mm = (int)(Math.abs(distance_mm) * COUNTS_PER_MM);
            speed = Math.abs(speed);

            switch (direction) {
                case FORWARD:
                    left_1.setTargetPosition(distance_mm);
                    left_2.setTargetPosition(distance_mm);
                    right_1.setTargetPosition(distance_mm);
                    right_2.setTargetPosition(distance_mm);
                    break;
                case BACKWARD:
                    left_1.setTargetPosition(-distance_mm);
                    left_2.setTargetPosition(-distance_mm);
                    right_1.setTargetPosition(-distance_mm);
                    right_2.setTargetPosition(-distance_mm);
                    break;
                case RIGHT:
                    left_1.setTargetPosition(distance_mm);
                    left_2.setTargetPosition(-distance_mm);
                    right_1.setTargetPosition(-distance_mm);
                    right_2.setTargetPosition(distance_mm);
                    break;
                case LEFT:
                    left_1.setTargetPosition(-distance_mm);
                    left_2.setTargetPosition(distance_mm);
                    right_1.setTargetPosition(distance_mm);
                    right_2.setTargetPosition(-distance_mm);
            }

            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            setPower(speed);

            while (linearOpMode.opModeIsActive() && left_1.isBusy() && left_2.isBusy() && right_1.isBusy() && right_2.isBusy() && (runtime.seconds() < timeout_seconds)) {
                double var = speed * (1.0 - Math.abs(gyro.getIntegratedZValue() - holdAngle) / ALIGNMENT_ENCODER_DRIVE);
                switch (direction) {
                    case FORWARD:
                        if (gyro.getIntegratedZValue() < holdAngle) {
                            right_2.setPower(var);
                            left_2.setPower(speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            left_2.setPower(var);
                            right_2.setPower(speed);
                        } else {
                            left_2.setPower(speed);
                            right_2.setPower(speed);
                        }
                        break;
                    case BACKWARD:
                        if (gyro.getIntegratedZValue() < holdAngle) {
                            left_1.setPower(var);
                            right_1.setPower(speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            right_1.setPower(var);
                            left_1.setPower(speed);
                        } else {
                            left_1.setPower(speed);
                            right_1.setPower(speed);
                        }
                        break;
                    case LEFT:
                        if (gyro.getIntegratedZValue() < holdAngle) {
                            left_1.setPower(var);
                            right_1.setPower(var);
                            left_2.setPower(speed);
                            right_2.setPower(speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            left_2.setPower(var);
                            right_2.setPower(var);
                            left_1.setPower(speed);
                            right_1.setPower(speed);
                        } else {
                            setPower(speed);
                        }
                        break;
                    case RIGHT:
                        if (gyro.getIntegratedZValue() < holdAngle) {
                            right_2.setPower(var);
                            left_2.setPower(var);
                            right_1.setPower(speed);
                            left_1.setPower(speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            right_1.setPower(var);
                            left_1.setPower(var);
                            right_2.setPower(speed);
                            left_2.setPower(speed);
                        } else {
                            setPower(speed);
                        }
                }
            }

            setPower(0.0);

            linearOpMode.telemetry.addData("Time", runtime.seconds());
            linearOpMode.telemetry.update();
            //linearOpMode.sleep(2000);

            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed, int angle) {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        if (linearOpMode.opModeIsActive()) {
            speed = Math.abs(speed);
            holdAngle += angle;

            if (angle > 0)
                while (linearOpMode.opModeIsActive() && (gyro.getIntegratedZValue() < holdAngle)) {
                    double var = (speed - ALIGNMENT_GYRO_TURN) * (holdAngle - gyro.getIntegratedZValue()) / angle + ALIGNMENT_GYRO_TURN;

                    left_1.setPower(var);
                    left_2.setPower(var);
                    right_1.setPower(-var);
                    right_2.setPower(-var);

                    linearOpMode.telemetry.addData("Angle", gyro.getIntegratedZValue());
                    linearOpMode.telemetry.update();
                }
            else if (angle < 0)
                while (linearOpMode.opModeIsActive() && (gyro.getIntegratedZValue() > holdAngle)) {
                    double var = (speed - ALIGNMENT_GYRO_TURN) * Math.abs(holdAngle - gyro.getIntegratedZValue()) / -angle + ALIGNMENT_GYRO_TURN;

                    left_1.setPower(-var);
                    left_2.setPower(-var);
                    right_1.setPower(var);
                    right_2.setPower(var);

                    linearOpMode.telemetry.addData("Angle", gyro.getIntegratedZValue());
                    linearOpMode.telemetry.update();
                }

            setPower(0.0);
        }
    }

    public void distanceDrive(double speed, Direction direction, double distance_cm, double timeout_seconds) {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        if (linearOpMode.opModeIsActive()) {
            speed = Math.abs(speed);

            switch (direction) {
                case FORWARD:
                    setPower(speed);
                    break;
                case LEFT:
                    left_1.setPower(-speed);
                    left_2.setPower(speed);
                    right_1.setPower(speed);
                    right_2.setPower(-speed);
                    break;
                case BACKWARD:
                    setPower(-speed);
            }

            runtime.reset();

            switch (direction) {
                case FORWARD:
                    while (linearOpMode.opModeIsActive() && (range_1.getDistance(DistanceUnit.CM) > distance_cm || runtime.seconds() < timeout_seconds)) {
                        double var = speed * (1.0 - Math.abs(gyro.getIntegratedZValue() - holdAngle) / ALIGNMENT_DISTANCE_DRIVE);

                        if (gyro.getIntegratedZValue() < holdAngle) {
                            right_2.setPower(var);
                            left_2.setPower(speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            left_2.setPower(var);
                            right_2.setPower(speed);
                        } else {
                            left_2.setPower(speed);
                            right_2.setPower(speed);
                        }
                    }
                    linearOpMode.telemetry.addData("Distance", range_1.getDistance(DistanceUnit.CM));
                    break;
                case BACKWARD:
                    while (linearOpMode.opModeIsActive() && (range_2.getDistance(DistanceUnit.CM) > distance_cm || runtime.seconds() < timeout_seconds)) {
                        double var = speed * (1.0 - Math.abs(gyro.getIntegratedZValue() - holdAngle) / ALIGNMENT_DISTANCE_DRIVE);

                        if (gyro.getIntegratedZValue() < holdAngle) {
                            left_1.setPower(-var);
                            right_1.setPower(-speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            right_1.setPower(-var);
                            left_1.setPower(-speed);
                        } else {
                            left_1.setPower(-speed);
                            right_1.setPower(-speed);
                        }
                    }
                    linearOpMode.telemetry.addData("Distance", range_2.getDistance(DistanceUnit.CM));
                    break;
                case LEFT:
                    while (linearOpMode.opModeIsActive() && (range_3.getDistance(DistanceUnit.CM) > distance_cm || runtime.seconds() < timeout_seconds)) {
                        double var = speed * (1.0 - Math.abs(gyro.getIntegratedZValue() - holdAngle) / ALIGNMENT_DISTANCE_DRIVE);

                        if (gyro.getIntegratedZValue() < holdAngle) {
                            left_1.setPower(-var);
                            right_1.setPower(var);
                            left_2.setPower(speed);
                            right_2.setPower(-speed);
                        } else if (gyro.getIntegratedZValue() > holdAngle) {
                            left_2.setPower(var);
                            right_2.setPower(-var);
                            left_1.setPower(-speed);
                            right_1.setPower(speed);
                        } else {
                            left_2.setPower(speed);
                            right_2.setPower(-speed);
                            left_1.setPower(-speed);
                            right_1.setPower(speed);
                        }
                    }
                    linearOpMode.telemetry.addData("Distance", range_3.getDistance(DistanceUnit.CM));
            }

            setPower(0.0);

            linearOpMode.telemetry.update();
            //linearOpMode.sleep(2000);
        }
    }
}
