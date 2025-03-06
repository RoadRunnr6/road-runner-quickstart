package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Base64;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.concurrent.TimeUnit;
import java.lang.Math;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import java.util.HashMap;


@TeleOp(name = "mainCodeV1")
public class testerScript extends LinearOpMode {
    // variable declaration
    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor horizontalExtender;



    private DcMotor intakeMotor;
    private DcMotor verticalExtender;
    private Servo clawServo;
    private Servo bucketServo;

    private Encoder servoEncoder;
    private double servoEncoder1Pos = 0;

    int horizontalExtenderMIN;
    int horizontalExtenderMAX;
    int verticalExtenderMIN;
    int verticalExtenderMAX;
    double armProportionality = 1;
    int lastVerticalPos = 0;
    double SERVOINCREMENT = 0.05;
    String currentTestGroup = "none";
    /*String currentTestSubdevice = "none"; */


    private void testServos(Servo[] servoArray, String[] servoTypes, int[] servoTestMovements){
        currentTestGroup = "Servos";
        HashMap<long[], Double> servoData = new HashMap<long[], Double>();
        for (int s = 0; s < servoArray.length; s++){
            Servo servoToTest = servoArray[s];
            double encoderStop = 1.5; //what encoder value to stop recording data at. I think somewhere around 1.5 is good because the servo should do less than 1 full rotation
            int lastDataPoint = 0;
            int dataPoint = 0;

            telemetry.addData("Testing Servo (port #): ", servoToTest.getPortNumber());
            telemetry.addData("Servo Type: ", servoTypes[s]);
            telemetry.addData("Moving Servo To:", servoTestMovements[s]);



            servoToTest.setPosition(servoTestMovements[s]);

            PositionVelocityPair data = servoEncoder.getPositionAndVelocity();



            while(servoEncoder1Pos <= encoderStop){

                long time = System.currentTimeMillis();

                if ( time - lastDataPoint >= 100 ){

                    long[] datapointTime = {dataPoint, time};
                    
                    servoData.put(datapointTime, servoEncoder1Pos);
                    dataPoint++;
                }

            }

        }
    }



    private void printThings() {
        telemetry.addData("Test Group: ", currentTestGroup);

    }


    private void hardwareMapping() {


        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        //colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
        clawServo = hardwareMap.get(Servo.class, "clawServo"); //add a servo onto the robot just to make sure this works (idk if this will error without one)
        verticalExtender = hardwareMap.get(DcMotor.class, "verticalExtender");
        horizontalExtender = hardwareMap.get(DcMotor.class, "horizontalExtender");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake/parallel");

        servoEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "servoEncoder1")));

        //servoTrackerEncoder1 = hardwareMap.get(DcMotor.class, "servoTracker1");


        Servo[] servoList = {
                clawServo,
                bucketServo
        };

        String[] servoTypes = {
                "torque",
                "torque"
        };

        int[] servoTestMovements = {
                1,
                1,
        };


    }

    private void horizontalExtenderSetup() {
        horizontalExtenderMIN = horizontalExtender.getCurrentPosition() - 100;
        //was 3000
        horizontalExtenderMAX = horizontalExtenderMIN - 1700;
        horizontalExtender.setTargetPosition(horizontalExtenderMIN);
        horizontalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalExtender.setPower(1);
    }

    private void verticalExtenderSetup() {
        verticalExtender.setPower(2);
        verticalExtender.setTargetPosition(0);
        verticalExtenderMIN = verticalExtender.getCurrentPosition();

        verticalExtenderMAX = verticalExtenderMIN - 4300;
        //was 4000
        verticalExtender.setTargetPosition(verticalExtender.getCurrentPosition());
        verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lastVerticalPos = verticalExtenderMIN;
    }




    private void setupServos() {
        bucketServo.setPosition(0.8);
        clawServo.setPosition(0.8); //need to change soon
    }

    private void setupChassis() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
        setupServos();
        horizontalExtenderSetup();
        verticalExtenderSetup();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        waitForStart();
        while (opModeIsActive()) {

            /*
            if (gamepad1.a && gamepad1.b) { //just press a and b together to start the search like it would in autonomous
                double searchOrigin = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                while (searchColor(searchOrigin)) {
                }
            } else {
                chassisMovement(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
            */


            printThings();
        }
    }
}