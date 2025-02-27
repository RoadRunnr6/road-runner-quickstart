package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.concurrent.TimeUnit;
import java.lang.Math;


@TeleOp(name = "mainCodeV1")
public class mainCodeV1 extends LinearOpMode {
    // variable declaration
    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor horizontalExtender;

    private DcMotor intakeMotor;
    private DcMotor verticalExtender;
    private ColorSensor colorDetector;
    private Servo clawServo;
    private Servo bucketServo;
    int horizontalExtenderMIN;
    int horizontalExtenderMAX;
    int verticalExtenderMIN;
    int verticalExtenderMAX;
    double armProportionality = 1;
    int targetedAngle = 1; //for block search

    double searchOrigin; //for block search
    boolean transferSW; //for transfer macro
    int INCREMENT = 400;
    int lastVerticalPos = 0;
    double SERVOINCREMENT = 0.05;
    
    boolean isInMacro = false;
    //all servo positioning stuff is from 0 - 1 (decimals included) and not in radians / degrees for some reason, 0 is 0 degrees, 1 is 320 (or whatever the servo max is) degrees
    //all our servos have 320 degrees of movement so i limited it so it wont collide with the horizontalExtender too much


    int transferMacroState = 0;

    private void hardwareMapping() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
        clawServo = hardwareMap.get(Servo.class, "clawServo"); //add a servo onto the robot just to make sure this works (idk if this will error without one)
        verticalExtender = hardwareMap.get(DcMotor.class, "verticalExtender");
        horizontalExtender = hardwareMap.get(DcMotor.class, "horizontalExtender");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake/parallel");
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

    private void horizontalExtension(boolean in, boolean out, int increment) {

        if (!isInMacro) {

            int horizontalExtenderPosition = horizontalExtender.getCurrentPosition();
            telemetry.addData("downPressed?", in);
            if (in) { // if (DPAD-down) is being pressed and if not yet the min
                horizontalExtenderPosition += increment;   // Position in
                horizontalExtenderPosition = Math.min(Math.max(horizontalExtenderPosition, horizontalExtenderMAX), horizontalExtenderMIN);  //clamp the values to be between min and max
            } else if (out) {  // if (DPAD-up) is being pressed and if not yet max
                horizontalExtenderPosition -= increment;   // Position Out
                horizontalExtenderPosition = Math.min(Math.max(horizontalExtenderPosition, horizontalExtenderMAX), horizontalExtenderMIN);  //clamp the values to be between min and max
            }
            horizontalExtender.setTargetPosition(horizontalExtenderPosition);
        }
    }

    private void verticalExtension(boolean in, boolean out, int increment) {

        if (!isInMacro) {
            int verticalExtenderPosition = verticalExtender.getCurrentPosition();
            if (in) { // if (DPAD-down) is being pressed and if not yet the min
                verticalExtenderPosition += increment;   // Position in
                verticalExtenderPosition = Math.min(Math.max(verticalExtenderPosition, verticalExtenderMAX), verticalExtenderMIN);  //clamp the values to be between min and max
                lastVerticalPos = verticalExtenderPosition;
            } else if (out) {  // if (DPAD-up) is being pressed and if not yet max
                verticalExtenderPosition -= increment;   // Position Out
                verticalExtenderPosition = Math.min(Math.max(verticalExtenderPosition, verticalExtenderMAX), verticalExtenderMIN);  //clamp the values to be between min and max
                lastVerticalPos = verticalExtenderPosition;
            }
            verticalExtender.setTargetPosition(lastVerticalPos);
        }
    }

    private void dropMacro(boolean activate){

        boolean atPosition = ((Math.abs(horizontalExtender.getCurrentPosition() - horizontalExtenderMIN)) < 30 && (Math.abs(verticalExtender.getCurrentPosition() - verticalExtenderMIN)) < 30 );

        if (activate && transferSW) {
            transferSW = false;

            isInMacro = true;
            
            transferMacroState += 1;

            if (transferMacroState > 1) {
                intakeMotor.setPower(0);
                transferMacroState = 0;
                bucketServo.setPosition(0.8);
                clawServo.setPosition(0.8);
                isInMacro = false;
            }

        } else if (!activate){
            transferSW = true;
        }

        if (transferMacroState == 1){
            horizontalExtender.setTargetPosition(horizontalExtenderMIN);
            verticalExtender.setTargetPosition(verticalExtenderMIN);
            bucketServo.setPosition(0.95);
            telemetry.addData("atPos:", atPosition);
            if (atPosition){
                clawServo.setPosition(0.3);
                isInMacro = false;
            }else{
                clawServo.setPosition(0.8);
            }
            lastVerticalPos = verticalExtenderMIN;

        }

    }


    private void setupServos() {
        bucketServo.setPosition(0.8);
        clawServo.setPosition(0.8); //need to change soon
    }

    private void setupChassis() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
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


    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void chassisMovement(float y, float x, float t) {
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        if (gamepad1.start) {
            imu.resetYaw();
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        rotX = x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI);
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        rotX = rotX * 1.1;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(rotY) + Math.abs(rotX) + Math.abs(t), 1));
        frontLeftPower = (rotY + rotX + t) / denominator;
        backLeftPower = (rotY - (rotX - t)) / denominator;
        frontRightPower = (rotY - (rotX + t)) / denominator;
        backRightPower = (rotY + (rotX - t)) / denominator;

        frontLeft.setPower(1 * frontLeftPower * armProportionality);
        backLeft.setPower(1 * backLeftPower * armProportionality);
        frontRight.setPower(1 * frontRightPower * armProportionality);
        backRight.setPower(1 * backRightPower * armProportionality);
    }

    private void bucketMovement(boolean down, boolean up, double increment) {
        if (!isInMacro) {
            double bucketPosition = bucketServo.getPosition();
            if (down) {
                bucketPosition -= increment;
            } else if (up) {
                bucketPosition += increment;
            }
            bucketPosition = clamp(bucketPosition, 0.55, 0.95);  //clamp the values to be between min and max
            bucketServo.setPosition(bucketPosition);
        }
    }


    private void clawMovement(boolean down, boolean up, double increment) {

        //if (transferMacroState == 0) {
            double clawPos = clawServo.getPosition();
            if (down) {
                clawPos -= 0.025;
            } else if (up) {
                clawPos += 0.025;
            }

            clawPos = clamp(clawPos, 0.2, 0.9);  //clamp the values to be between min and max
            clawServo.setPosition(clawPos);
        //}
    }

    private void intakeMotorControl(double lTrigger, double rTrigger){

        /*
        if (lTrigger >= 0.1 || rTrigger >= 0.1) {
            if (lTrigger > rTrigger) {
                intakeMotor.setPower(-lTrigger);

            } else if (rTrigger > lTrigger) {
                intakeMotor.setPower(rTrigger);

            }
        } else {
            intakeMotor.setPower(0);
        }
        */

         intakeMotor.setPower(rTrigger - lTrigger);
    }

    private void printThings() {
        //telemetry.addData("Color: ", colorDetection());
        //telemetry.addData("difference", distanceBetweenAngles((float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 90f));
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("horizontalExtenderPosition", horizontalExtender.getCurrentPosition());
        //telemetry.addData("horizontalExtenderMax", horizontalExtenderMAX);
        telemetry.addData("claw angle: ", clawServo.getPosition());
        telemetry.addData("bucket position:", bucketServo.getPosition());
        telemetry.addData("VerticalExtenderFromPrintFunc:", verticalExtender.getCurrentPosition());
        //telemetry.addData("lTrigger", gamepad1.left_trigger);
        //telemetry.addData("rTrigger", gamepad1.right_trigger);
        telemetry.addData("Macro", transferMacroState);

        telemetry.update();
    }

    //rotation which way you need to turn, and how much you need to turn to get to target angle
    private static float distanceBetweenAngles(float alpha, float beta) {
        float phi = (beta - alpha) % 360; // Raw difference in range [-359, 359]
        float distance = phi > 180 ? phi - 360 : (phi < -180 ? phi + 360 : phi);
        return distance;
    }

    private boolean searchColor(double searchOrigin) throws InterruptedException {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        float directionBetweenAngles;
        if (colorDetection().equals("Yellow") || colorDetection().equals("Red")) {
            float power = -0.3f*targetedAngle;
            TimeUnit.SECONDS.sleep(1);
            while (!(colorDetection().equals("Yellow") || colorDetection().equals("Red"))) {
                chassisMovement(0,0, power);
            }
            return false;
        } else {
            directionBetweenAngles = distanceBetweenAngles((float)botHeading, (float)(30*targetedAngle + searchOrigin));
            float VELOCITYTANGENTIAL = 1000; //unsure what the units are for this
            float rotationSpeed;
            if (horizontalExtender.getCurrentPosition() > -1000) {
                rotationSpeed = 0.9f; //no easing in the beginning
            } else {
                rotationSpeed = (VELOCITYTANGENTIAL/(-(horizontalExtender.getCurrentPosition() + 200))); //rotationSpeed (omega) = Vt/r where R is horizontalExtenderMAX ~ 3000
            }
            int velocityhorizontalExtender = (int)(10 * ((215 * rotationSpeed)/(60f)));
            while (horizontalExtender.getCurrentPosition() > -500) {
                horizontalExtension(false, true, INCREMENT);
            }
            horizontalExtension(false, true, velocityhorizontalExtender);
            rotateTo((30*targetedAngle) + searchOrigin, rotationSpeed);
            if (Math.abs(directionBetweenAngles) < 4) { //determines if the robot is facing a direction
                if (targetedAngle == 1) { //if it was turning one way, switch it
                    targetedAngle = -1;
                } else {
                    targetedAngle = 1;
                }
            }
        }
        return (horizontalExtender.getCurrentPosition() >= horizontalExtenderMAX + 100);
        // extend horizontalExtender if not already extended
        // extend to stage 1. Closest 2. Medium 3. Far
        // rotate x degrees
        // if target color is detected then finish
        // activate claw and pick up
    }

    private String colorDetection() {
        String[] colors = {"Yellow", "Blue", "Red"};
        String color = "";
        float ratioGreenOverRed = ((float)colorDetector.green() / colorDetector.red());
        float ratioBlueOverRed = ((float)colorDetector.blue() / colorDetector.red());

        if ((ratioGreenOverRed >= 1.1 && ratioGreenOverRed <= 2.0) &&
                (ratioBlueOverRed >= 0.1 && ratioBlueOverRed <= 0.8)) {
            color = colors[0]; // Yellow
        }
        if ((ratioGreenOverRed >= 1.5 && ratioGreenOverRed <= 2.7) &&
                (ratioBlueOverRed >= 2.0 && ratioBlueOverRed <= 10.0)) {
            color = colors[1]; // Blue
        }
        if ((ratioGreenOverRed >= 0.2 && ratioGreenOverRed <= 1) &&
                (ratioBlueOverRed >= 0.1 && ratioBlueOverRed <= 0.8)) {
            color = colors[2]; // Red
        }
        return color;
    }

    private void rotateTo(double targetDegree, float maxRotationSpeed) {
        double botHeading;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        float direction = distanceBetweenAngles((float)botHeading, (float)targetDegree);
        float power = Math.max(Math.min(maxRotationSpeed, (float)(0.001 * Math.pow(direction, 2))), 0.225f); // 1 is clockwise, -1 is counterclock minimum is 0.1 (might need to be lower) and max is 0.5
        if (Math.abs(direction) < 1f) {     // if the angle is less than 1 then poweroff
            power = 0f;
        }
        power = direction < 0 ? power * -1: power;
        chassisMovement(0,0, power);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && gamepad1.b) { //just press a and b together to start the search like it would in autonomous
                double searchOrigin = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                while (searchColor(searchOrigin)) {
                }
            } else {
                chassisMovement(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
            /*
            horizontalExtension(gamepad1.dpad_down,gamepad1.dpad_up,INCREMENT);
            bucketMovement(gamepad2.a, gamepad2.y, SERVOINCREMENT);
            clawMovement(gamepad2.dpad_up, gamepad2.dpad_down, SERVOINCREMENT);
            verticalExtension(gamepad1.a, gamepad1.y, INCREMENT); //gamepad1.x is assigned switchVerticalPosition where if that is true, we are switching whether the extender goes up or down, true is up and false is down
            intakeMotorControl(gamepad2.left_trigger, gamepad2.right_trigger);
            dropMacro(gamepad2.b, 0.95,0.3);

             */


            horizontalExtension(gamepad1.dpad_down,gamepad1.dpad_up,INCREMENT);
            bucketMovement(gamepad2.y, gamepad2.a, 0.025);
            clawMovement(gamepad2.dpad_up, gamepad2.dpad_down, SERVOINCREMENT);
            verticalExtension(gamepad1.a, gamepad1.y, INCREMENT); //gamepad1.x is assigned switchVerticalPosition where if that is true, we are switching whether the extender goes up or down, true is up and false is down
            intakeMotorControl(gamepad2.left_trigger, gamepad2.right_trigger);
<<<<<<< HEAD
            dropMacro(gamepad2.b);

=======
            dropAutoPosition(gamepad2.b);
            if (verticalExtender.getCurrentPosition() < -3000) {
                armProportionality = (1/Math.log10(1300))*(Math.log10(4311+(verticalExtender.getCurrentPosition())));
            }
            else {
                armProportionality = 1;
            }
>>>>>>> ebfc1066f64de9ce0b3580e0dd7b154e0b052776

            printThings();
        }
    }
}