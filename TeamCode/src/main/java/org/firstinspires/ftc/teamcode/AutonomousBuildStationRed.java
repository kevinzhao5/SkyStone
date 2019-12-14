package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousBuildStationRed", group="Autonomous")
public class AutonomousBuildStationRed extends LinearOpMode {

    //Objects
    ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor leftBack; //port 3
    DcMotor leftFront; //port 0
    DcMotor rightBack; //port 2
    DcMotor rightFront; //port 1
    DcMotor rnpUp1; //port 1
    DcMotor rnpUp2; //port 2

    //Servos
    Servo intakeLeft; //port 1
    Servo intakeRight; //port 0
    Servo extension; //port 2

    /**
     *
     */
    @Override
    public void runOpMode() {

        //Initialize DcMotors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rnpUp1 = hardwareMap.get(DcMotor.class, "rnpUp1");
        rnpUp2 = hardwareMap.get(DcMotor.class, "rnpUp2");

        //Initialize Servos
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        extension = hardwareMap.get(Servo.class, "extension");

        //Set zero power behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rnpUp1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rnpUp2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set directions of the motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
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

        waitForStart();



    }

    /**
     *
     * @param power
     */
    public void setAllDriveMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

}