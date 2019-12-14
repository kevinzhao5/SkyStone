package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake_Test", group = "OpMode")
public class Intake_Test extends OpMode {

    //Objects
    ElapsedTime runtime;

    //Servos
    Servo clawLeft;
    Servo clawRight;

    @Override
    public void init() {

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        clawRight.setDirection(Servo.Direction.FORWARD);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {

        runtime.reset();

    }


    @Override
    public void loop(){

        if(gamepad1.right_stick_y > 0)
        {
            clawRight.setPosition(1);
            clawLeft.setPosition(1);
        }
        else if(gamepad1.right_stick_y < 0) {
            clawRight.setPosition(0);
            clawLeft.setPosition(0);
        }
        else {
            clawRight.setPosition(0.5);
            clawLeft.setPosition(0.5);
        }

        //Display data
        telemetry.addData("Runtime: ", getRuntime());

    }
}
