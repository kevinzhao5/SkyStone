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
    Servo intakeLeft;
    Servo intakeRight;

    @Override
    public void init() {

        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");

        intakeLeft.setDirection(Servo.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.FORWARD);

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {

        runtime.reset();

    }


    @Override
    public void loop() {

        //Control servos for intake
        if(gamepad2.right_stick_y > 0)
        {
            intakeRight.setPosition(1);
            intakeLeft.setPosition(1);
        }
        else if(gamepad2.right_stick_y < 0) {
            intakeRight.setPosition(0);
            intakeLeft.setPosition(0);
        }
        else {
            intakeRight.setPosition(0.5);
            intakeLeft.setPosition(0.5);
        }

        //Display data
        telemetry.addData("Runtime: ", getRuntime());

    }
}
