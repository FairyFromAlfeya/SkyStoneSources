package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Units.Chassis;
import org.firstinspires.ftc.teamcode.Units.Collector;
import org.firstinspires.ftc.teamcode.Units.Grabber;

@TeleOp(name = "TeleOp", group = "Tele")
//@Disabled
public class Tele extends OpMode {
    private final static double FAST_SPEED_COEFF = 1.0;
    private final static double SLOW_SPEED_COEFF = 0.2;

    private Chassis chassis = new Chassis();
    private Grabber grabber = new Grabber();
    private Collector collector = new Collector();

    private boolean bPrev = false, bCurr;
    private boolean yPrev = false, yCurr;
    private boolean aPrev = false, aCurr;
    private boolean aBool = false, yBool = false;

    private double speedCoeff = FAST_SPEED_COEFF;

    @Override
    public void init() {
        chassis.init(this);
        grabber.init(this);
        collector.init(this);
    }

    @Override
    public void loop() {
        //---------------------------------Driving---------------------------------//
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        bCurr = gamepad1.right_stick_button;

        if (bCurr && !bPrev)
            if (speedCoeff != SLOW_SPEED_COEFF)
                speedCoeff = SLOW_SPEED_COEFF;
            else
                speedCoeff = FAST_SPEED_COEFF;

        bPrev = bCurr;

        double left_1_spd = Range.clip(y + x + turn, -1.0, 1.0) * speedCoeff;
        double left_2_spd = Range.clip(y - x + turn, -1.0, 1.0) * speedCoeff;
        double right_1_spd = Range.clip(y - x - turn, -1.0, 1.0) * speedCoeff;
        double right_2_spd = Range.clip(y + x - turn, -1.0, 1.0) * speedCoeff;

        chassis.setPower(left_1_spd, left_2_spd, right_1_spd, right_2_spd);
        //-------------------------------------------------------------------------//

        //--------------------------------Collector--------------------------------//
        collector.getCollector1().setPower(-gamepad2.left_stick_y);
        collector.getCollector2().setPower(gamepad2.left_stick_y);

        yCurr = gamepad2.y;

        if (yCurr && !yPrev) {
            if (yBool)
                collector.getBlock().setPosition(0.9);
            else
                collector.getBlock().setPosition(0.5);

            yBool = !yBool;
        }

        yPrev = yCurr;
        //-------------------------------------------------------------------------//

        //-----------------------------------Lift----------------------------------//
        collector.getMotorTurn().setPower(-gamepad2.right_stick_y);

        if (gamepad2.right_bumper)
            collector.getCapture().setPosition(0.25);
        else if (gamepad2.left_bumper)
            collector.getCapture().setPosition(1);

        aCurr = gamepad2.a;

        if (aCurr && !aPrev) {
            if (aBool)
                collector.getTurnHand().setPosition(0.95);
            else
                collector.getTurnHand().setPosition(0.05);

            aBool = !aBool;
        }

        aPrev = aCurr;
        //-------------------------------------------------------------------------//

        //--------------------------------Foundation-------------------------------//
        if (gamepad1.right_bumper)
            grabber.lockFundament();
        else if (gamepad1.left_bumper)
            grabber.unlockFundament();
        //-------------------------------------------------------------------------//

        telemetry.addData("Slow mode", speedCoeff == SLOW_SPEED_COEFF ? "on" : "off");
    }
}
