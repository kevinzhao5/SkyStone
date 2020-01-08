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
        Left joystick to move the claw arm up and down
        Right joystick (move in y direction) to spin intake wheels

*/

@TeleOp(name="WheelIntakeDrive", group="OpMode")
public class WheelIntakeDrive extends OpMode{

    //Objects
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor leftBack; //port 3
    DcMotor leftFront; //port 0
    DcMotor rightBack; //port 2
    DcMotor rightFront; //port 1

    //Servos
    Servo intakeLeft; //port 1
    Servo intakeRight; //port 0
    Servo rnp; //port 2

    //Variables
    double speedMultiplier = 1;
    boolean aPressed = false;

    @Override
    public void init() {

        //Initialize DcMotors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        //Initialize Servos
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        rnp = hardwareMap.get(Servo.class, "rnp");

        //Set zero power behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set directions of the motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        //Set direction of the Servos
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.FORWARD);
        rnp.setDirection(Servo.Direction.FORWARD);

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
            setAllDriveMotorPower(-gamepad1.right_stick_x * speedMultiplier);
        }

        //If A is not pressed
        if (!gamepad1.a) {
            aPressed = false;
        }

        //Toggle the speed multiplier
        if (gamepad1.a && !aPressed) {
            speedMultiplier = 1.3 - speedMultiplier;
            aPressed = true;
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

        if(gamepad2.left_stick_y > 0)
        {
            rnp.setPosition(1);
        }
        else if(gamepad2.left_stick_y < 0) {
            rnp.setPosition(0);
        }
        else {
            rnp.setPosition(0.5);
        }

        //Display data
        telemetry.addData("Runtime: ", getRuntime());
    }

    public void setAllDriveMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

}