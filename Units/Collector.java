package org.firstinspires.ftc.teamcode.Units;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {

    private DcMotor collector1;
    private DcMotor collector2;
    private DcMotor motorTurn;

    private Servo capture;
    private Servo turnHand;
    private Servo block;

    public void init(OpMode opMode) {
        collector1 = opMode.hardwareMap.get(DcMotor.class, "collector1");
        collector2 = opMode.hardwareMap.get(DcMotor.class, "collector2");
        motorTurn = opMode.hardwareMap.get(DcMotor.class, "MotorTurn");
        capture = opMode.hardwareMap.get(Servo.class, "capture");
        turnHand = opMode.hardwareMap.get(Servo.class, "turnhand1");
        block = opMode.hardwareMap.get(Servo.class, "block");

        collector1.setDirection(DcMotorSimple.Direction.FORWARD);
        collector2.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTurn.setDirection(DcMotorSimple.Direction.FORWARD);

        collector1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getCollector1() {
        return collector1;
    }

    public DcMotor getCollector2() {
        return collector2;
    }

    public DcMotor getMotorTurn() {
        return motorTurn;
    }

    public Servo getCapture() {
        return capture;
    }

    public Servo getTurnHand() {
        return turnHand;
    }

    public Servo getBlock() {
        return block;
    }
}
