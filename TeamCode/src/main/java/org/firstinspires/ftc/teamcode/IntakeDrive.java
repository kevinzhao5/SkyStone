package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="intake tester", group="OpMode")
public class IntakeDrive extends OpMode {


    //Objects
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor leftBack; //port 3
    DcMotor leftFront; //port 0
    DcMotor rightBack; //port 2
    DcMotor rightFront; //port 1

    //Variables
    double speedMultiplier;
    boolean aPressed;

    @Override
    public void init() {

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);


        //Initialize DcMotors
        /*
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

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

         */
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


        if(gamepad2.right_stick_y < 0)
        {
            leftBack.setPower(-1);
            leftFront.setPower(-1);
        }
        else if(gamepad2.right_stick_y > 0)
        {
            leftBack.setPower(1);
            leftFront.setPower(1);
        }
        else
        {
            leftBack.setPower(0);
            leftFront.setPower(0);
        }


        //Normalize the values if the sum is greater than one to fit motor power
        /*double x = gamepad1.left_stick_x;
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

        //Display runtime
        telemetry.addData("Runtime: ", getRuntime());
        telemetry.addData("x: ", x);
        telemetry.addData("y: ", y);
        telemetry.addData("speed multiplier: ", speedMultiplier);
        */
    }

    public void setAllDriveMotorPower(double power) {
        leftFront.setPower(power);
        //rightFront.setPower(power);
        leftBack.setPower(power);
        //rightBack.setPower(power);
    }

}