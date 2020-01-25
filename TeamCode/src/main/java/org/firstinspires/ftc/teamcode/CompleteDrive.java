package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*

    Controlling the robot:

    Gamepad1(Driving):
        Left joystick to move the robot
        Right joystick to turn
        Press A to toggle speed multiplier

    Gamepad2(Intake):
        Left joystick (move in y direction) to spin intake wheels
        Right joystick (move in y direction) to move hooks

*/

@TeleOp(name="CompleteDrive", group="OpMode")
public class CompleteDrive extends OpMode{

    //Objects
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor leftBack; //port 3
    DcMotor leftFront; //port 0
    DcMotor rightBack; //port 2
    DcMotor rightFront; //port 1

    DcMotor leftWheel; //port 2
    DcMotor rightWheel; //port 1

    //Servos
    Servo leftHook; //port 0
    Servo rightHook; //port 1
    Servo blockPush; // port 2

    //Variables
    double speedMultiplier;
    boolean aPressed;

    @Override
    public void init() {

        //Initialize DcMotors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");

        //Initialize servos
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        blockPush = hardwareMap.get(Servo.class, "blockPush");


        //Set zero power behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set directions of the motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        //Set direction of the Servos
        leftHook.setDirection(Servo.Direction.REVERSE);
        rightHook.setDirection(Servo.Direction.FORWARD);
        rightHook.setDirection(Servo.Direction.FORWARD);

        //Initialize the variables
        speedMultiplier = 1;
        aPressed = false;

        //Tell user that initialization is complete
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Drive the robot

        //Normalize the values if the sum is greater than one to fit motor power
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double sum = Math.abs(x) + Math.abs(y);

        if (sum > 1) {
            double newx = x / sum, newy = y / sum;
            x = newx;
            y = newy;
        }

        x *= speedMultiplier;
        y *= speedMultiplier;

        //Driving
        leftFront.setPower(-x + y);
        rightFront.setPower(-x - y);
        leftBack.setPower(x + y);
        rightBack.setPower(x - y);

        //Turning
        if (Math.abs(gamepad1.right_stick_x) >= 0.000001) {
            setAllDriveMotorPower(-gamepad1.right_stick_x);
        }

        //If A is not pressed
        if (!gamepad1.a) {
            aPressed = false;
        }

        //Toggle the speed multiplier
        if (gamepad1.a && !aPressed) {
            speedMultiplier = 1.2 - speedMultiplier;
            aPressed = true;
        }

        //Control wheels
        leftWheel.setPower(gamepad2.left_stick_y);
        rightWheel.setPower(gamepad2.left_stick_y);

        //Control hooks
        if (gamepad2.right_bumper) {
            leftHook.setPosition(1);
            rightHook.setPosition(1);
        } else {
            leftHook.setPosition(0);
            rightHook.setPosition(0);
        }


        if(gamepad2.a)
        {
            blockPush.setPosition(0);
        }
        else
        {
            blockPush.setPosition(0.5);
        }
        //Display runtime
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("speed multiplier: ", speedMultiplier);

    }

    public void setAllDriveMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

}