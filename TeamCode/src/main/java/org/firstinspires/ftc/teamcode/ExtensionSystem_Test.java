package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ExtensionSystem_Test", group = "OpMode")
public class ExtensionSystem_Test extends OpMode {

    //Objects
    ElapsedTime runtime;

    //Servos
    Servo extension;

    @Override
    public void init() {

        extension = hardwareMap.get(Servo.class, "extension");

        extension.setDirection(Servo.Direction.REVERSE);

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {

        runtime.reset();

    }


    @Override
    public void loop() {

        if (gamepad2.left_bumper) {
            extension.setPosition(0);
        } else if (gamepad2.right_bumper) {
            extension.setPosition(1);
        } else {
            extension.setPosition(0.5);
        }
        //Display data
        telemetry.addData("Runtime: ", getRuntime());

    }
}
