package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ArmAndIntake_Test", group="OpMode")
public class ArmAndIntake_Test extends OpMode{

    //Objects
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor rnpUp1;
    DcMotor rnpUp2;

    //Servos
    Servo intakeLeft;
    Servo intakeRight;
    Servo extension;

    //Variables
    int minPos = 0;
    int maxPos = 3800;

    boolean downPressed = false;
    boolean upPressed = false;
    boolean leftPressed = false;
    boolean rightPressed = false;

    @Override
    public void init() {

        //Initialize DcMotors
        rnpUp1 = hardwareMap.get(DcMotor.class, "rnpUp1");
        rnpUp2 = hardwareMap.get(DcMotor.class, "rnpUp2");

        //Initialize Servos
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        extension = hardwareMap.get(Servo.class, "extension");

        //Set zero power behavior
        rnpUp1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rnpUp2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set directions of the motors
        rnpUp1.setDirection(DcMotor.Direction.REVERSE);
        rnpUp2.setDirection(DcMotor.Direction.FORWARD);

        //Set direction of the Servos
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.FORWARD);
        extension.setDirection(Servo.Direction.REVERSE);

        //Set run mode
        rnpUp1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rnpUp2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Tell user that initialization is complete
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Move the arm up and down
        int position1 = rnpUp1.getCurrentPosition();
        int position2 = rnpUp2.getCurrentPosition();
        double pwr = -gamepad2.left_stick_y;
        int newPos1 = (int) (pwr * 200) + position1;
        if (minPos < newPos1 && newPos1 < maxPos) {
            rnpUp1.setTargetPosition(newPos1);
            rnpUp1.setPower(pwr);
        }

        int newPos2 = (int) (pwr*200) + position2;
        if (minPos < newPos2 && newPos2 < maxPos) {
            rnpUp2.setTargetPosition(newPos2);
            rnpUp2.setPower(pwr);
        }

        //Change limits
        if (gamepad2.dpad_down && !downPressed) {
            maxPos -= 100;
            downPressed = true;
        }
        if (!gamepad2.dpad_down) {
            downPressed = false;
        }

        if (gamepad2.dpad_up && !upPressed) {
            maxPos += 100;
            upPressed = true;
        }
        if (!gamepad2.dpad_up) {
            upPressed = false;
        }

        if (gamepad2.dpad_left && !leftPressed) {
            minPos -= 100;
            leftPressed = true;
        }
        if (!gamepad2.dpad_left) {
            leftPressed = false;
        }

        if (gamepad2.dpad_right && !rightPressed) {
            minPos += 100;
            rightPressed = true;
        }
        if (!gamepad2.dpad_right) {
            rightPressed = false;
        }

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

        //Control the extension system
        if (gamepad2.left_bumper) {
            extension.setPosition(0);
        } else if (gamepad2.right_bumper) {
            extension.setPosition(1);
        } else {
            extension.setPosition(0.5);
        }

        //Display data
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("min pos: ", minPos);
        telemetry.addData("max pos: ", maxPos);

        telemetry.addData("position 1: ", position1);
        telemetry.addData("new pos 1: ", newPos1);

        telemetry.addData("position 2: ", position2);
        telemetry.addData("new pos 2: ", newPos2);
    }

}