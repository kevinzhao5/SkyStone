package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonBlueStones", group="Autonomous")
public class AutonBlueStones extends LinearOpMode {

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

    //Constants
    final double secondsPerCm = 0.00632807994;
    final double secondsPerDegree = 0.00515;

    @Override
    public void runOpMode() {

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

        //Tell user that initialization is complete
        telemetry.addData("Status", "Initialized");

        waitForStart();

        driveBackward(15, 0.5);

        pause(0.2);

        driveRight(100, 0.2);

        pause(0.2);

        leftHook.setPosition(1);
        rightHook.setPosition(1);

        pause(0.8);

        driveLeft(50, 0.5);

        pause(0.2);

        driveRight(10, 0.5);

        pause(0.2);

        driveForward(150, 0.5);

        pause(0.2);

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        pause(0.2);

        driveLeft(10, 0.5);

        pause(0.2);

        driveBackward(70, 0.5);

        pause(0.2);

    }

    public void setAllDriveMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void turnRight(double degrees) {
        setAllDriveMotorPower(-1);
        pause(degrees * secondsPerDegree);
        setAllDriveMotorPower(0);
    }

    public void turnLeft(double degrees) {
        setAllDriveMotorPower(1);
        pause(degrees * secondsPerDegree);
        setAllDriveMotorPower(0);
    }

    public void drive(double x, double y) {

        //Normalize the values if the sum is greater than one to fit motor power
        double sum = Math.abs(x) + Math.abs(y);

        if (sum > 1) {
            double newx = x / sum, newy = y / sum;
            x = newx;
            y = newy;
        }

        //Driving
        leftFront.setPower(-x + y);
        rightFront.setPower(-x - y);
        leftBack.setPower(x + y);
        rightBack.setPower(x - y);
    }

    public void pause(double s) {
        long initTime = System.nanoTime();
        s *= 1000000000;
        while (System.nanoTime() < initTime + s) {

        }
    }

    public void driveForward(double cm, double power) {
        drive(0, -power);
        pause(cm * secondsPerCm * (1 / power));
        setAllDriveMotorPower(0);
    }

    public void driveBackward(double cm, double power) {
        drive(0, power);
        pause(cm * secondsPerCm * (1 / power));
        setAllDriveMotorPower(0);
    }

    public void driveLeft(double cm, double power) {
        drive(-power, 0);
        pause(cm * secondsPerCm * (1 / power));
        setAllDriveMotorPower(0);
    }

    public void driveRight(double cm, double power) {
        drive(power, 0);
        pause(cm * secondsPerCm * (1 / power));
        setAllDriveMotorPower(0);
    }

}