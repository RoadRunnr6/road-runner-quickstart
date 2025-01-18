package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;



@Autonomous(name = "redAuto", group = "Autonomous")
public class redAutoLeft extends LinearOpMode {
    // variable declaration
    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor arm;

    private DcMotor intakeMotor;
    private DcMotor verticalExtender;
    private ColorSensor colorDetector;
    private Servo clawServo;
    boolean verticalExtensionDirection = true;
    boolean xPressed = false;
    private Servo bucketServo;
    int ARMMIN;
    int ARMMAX;
    int EXTENDERMIN;
    int EXTENDERMAX;
    int targetedAngle = 1; //for block search
    double searchOrigin; //for block search
    int INCREMENT;
    double SERVOINCREMENT = 0.05;
    //all servo positioning stuff is from 0 - 1 (decimals included) and not in radians / degrees for some reason, 0 is 0 degrees, 1 is 320 (or whatever the servo max is) degrees
    //all our servos have 320 degrees of movement so i limited it so it wont collide with the arm too much
    private double servoMax = 1; //maximum angle the claw servo is allowed to move
    private double servoMin = 0; //minimum angle the claw servo is allowed to move

    private double clawYPos = 0.5; //uses this value to set the initial claw position in the middle of the max and min
    //i am using a variable because .getPosition() only returns the last position the servo was told to move, not its actual location\

    boolean isAutoPositioning = false;


    private void hardwareMapping() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
        clawServo = hardwareMap.get(Servo.class, "clawServo"); //add a servo onto the robot just to make sure this works (idk if this will error without one)
        verticalExtender = hardwareMap.get(DcMotor.class, "verticalExtender");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

    }

    private void armSetup() {
        arm.setPower(1);
        ARMMIN = arm.getCurrentPosition() - 3;
        //was 3000
        ARMMAX = ARMMIN - 3000;
        INCREMENT = 250;
    }

    private void extenderSetup() {
        verticalExtender.setPower(1);

        EXTENDERMIN = verticalExtender.getCurrentPosition();

        //was 4000

        EXTENDERMAX = EXTENDERMIN - 4400;
    }


    private void setupServos() {
        bucketServo.setPosition(0.5);
        clawServo.setPosition(0.5);
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
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo.setPosition(clawYPos);
    }

    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
        armSetup();
        extenderSetup();
    }

    private void postStartSetUp() {
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalExtender.setTargetPosition(verticalExtender.getCurrentPosition());
        verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void chassisMovement(float y, float x, float t, IMU imu, DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        rotX = x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI);
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        rotX = rotX * 1.1;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(rotY) + Math.abs(rotX) + Math.abs(t), 1));
        frontLeftPower = (rotY + rotX + t) / denominator;
        backLeftPower = (rotY - (rotX - t)) / denominator;
        frontRightPower = (rotY - (rotX + t)) / denominator;
        backRightPower = (rotY + (rotX - t)) / denominator;

        frontLeft.setPower(0.75 * frontLeftPower);
        backLeft.setPower(0.75 * backLeftPower);
        frontRight.setPower(0.75 * frontRightPower);
        backRight.setPower(0.75 * backRightPower);
    }

    private void intakeArmMovement(boolean in, boolean out, int increment, DcMotor horizontalExtender, double armMin, double armMax) {
        int extenderPosition = horizontalExtender.getCurrentPosition();
        if (in) {       // if (DPAD-down) is being pressed and if not yet the min
            extenderPosition += increment;   // Position in
        } else if (out) {  // if (DPAD-up) is being pressed and if not yet max
            extenderPosition -= increment;   // Position Out
        }
        extenderPosition = Math.max(Math.min(extenderPosition, ARMMIN), ARMMAX);  //clamp the values to be between min and max
        horizontalExtender.setTargetPosition(extenderPosition);
    }

    private void verticalExtension(boolean switchVerticalPosition) {

        if (switchVerticalPosition) {
            if (!xPressed) {
                xPressed = true;
                verticalExtensionDirection = !verticalExtensionDirection;
                if (verticalExtensionDirection) { //true = up
                    verticalExtender.setTargetPosition(EXTENDERMAX);
                }
                if (!verticalExtensionDirection) { //false = down
                    verticalExtender.setTargetPosition(EXTENDERMIN);
                }
            }
        } else {
            xPressed = false;
        }
    }

    private void bucketMovement(boolean down, boolean up, double increment) {
        if (!isAutoPositioning) {
            int max = 1;
            int min = 0;
            double bucketPosition = bucketServo.getPosition();
            if (down) {
                bucketPosition -= increment;
            } else if (up) {
                bucketPosition += increment;
            }
            bucketPosition = clamp(bucketPosition, min, max);  //clamp the values to be between min and max
            bucketServo.setPosition(bucketPosition);
        }
    }


    private void clawMovement(boolean down, boolean up, double increment) {
        double clawPos = clawServo.getPosition();
        if (down) {
            clawPos -= increment;
        } else if (up) {
            clawPos += increment;
        }
        clawPos = clamp(clawPos, servoMin, servoMax);  //clamp the values to be between min and max
        clawServo.setPosition(clawPos);
    }

    private void intakeMotorControl(boolean lTrigger, boolean rTrigger) {

        if (lTrigger || rTrigger) {
            if (lTrigger) {
                intakeMotor.setPower(1);
            } else if (rTrigger) {
                intakeMotor.setPower(-1);
            }
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void printThings() {
        telemetry.addData("Color: ", colorDetection(colorDetector));
        telemetry.addData("difference", distanceBetweenAngles((float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 90f));
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("armPosition", arm.getCurrentPosition());
        telemetry.addData("armMax", ARMMAX);
        telemetry.addData("claw angle: ", clawServo.getPosition());
        telemetry.addData("bucket postion:", bucketServo.getPosition());
        telemetry.addData("VerticalExtenderFromPrintFunc:", verticalExtender.getCurrentPosition());
        telemetry.addData("lTrigger", gamepad2.left_bumper);
        telemetry.addData("rTrigger", gamepad2.right_bumper);
        telemetry.update();
    }


    private String colorDetection(ColorSensor colorDetector) {
        String[] colors = {"Yellow", "Blue", "Red"};
        String color = "";
        float ratioGreenOverRed = ((float) colorDetector.green() / colorDetector.red());
        float ratioBlueOverRed = ((float) colorDetector.blue() / colorDetector.red());

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

    private void rotateTo(double targetDegree, float maxRotationSpeed, IMU imu, DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        double botHeading;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        float direction = distanceBetweenAngles((float) botHeading, (float) targetDegree);
        float power = Math.max(Math.min(maxRotationSpeed, (float) (0.001 * Math.pow(direction, 2))), 0.225f); // 1 is clockwise, -1 is counterclock minimum is 0.1 (might need to be lower) and max is 0.5
        if (Math.abs(direction) < 1f) {     // if the angle is less than 1 then poweroff
            power = 0f;
        }
        power = direction < 0 ? power * -1 : power;
        chassisMovement(0, 0, power, imu, backLeft, backRight, frontLeft, frontRight);
    }

    //rotation which way you need to turn, and how much you need to turn to get to target angle
    private float distanceBetweenAngles(float alpha, float beta) {
        float phi = (beta - alpha) % 360; // Raw difference in range [-359, 359]
        float distance = phi > 180 ? phi - 360 : (phi < -180 ? phi + 360 : phi);
        return distance;
    }


    public class BucketMovement {
        private Servo bucketServo;

        public BucketMovement(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        public class BucketUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(1); //alter as necessary
                return false;
            }
        }

        public Action bucketUp() {
            return new BucketUp();
        }

        public class BucketDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(0); //alter as necessary
                return false;
            }
        }

        public Action bucketDown() {
            return new BucketDown();
        }
    }

    public class VerticalExtension {
        private DcMotorEx verticalExtender;
        private boolean initialized = false;
        int EXTENDERMIN;

        public VerticalExtension(HardwareMap hardwareMap) {
            verticalExtender = hardwareMap.get(DcMotorEx.class, "verticalExtender");
            verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalExtender.setTargetPosition(verticalExtender.getCurrentPosition());
            verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class MoveUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    verticalExtender.setPower(0.8);
                    initialized = true;
                }
                EXTENDERMIN = verticalExtender.getCurrentPosition();
                EXTENDERMAX = EXTENDERMIN - 4400;
                verticalExtender.setTargetPosition(EXTENDERMAX);
                return false;
            }
        }

        public Action moveUp() {
            initialized = false;
            return new MoveDown();
        }

        public class MoveDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    verticalExtender.setPower(0.8);
                    initialized = true;
                }
                verticalExtender.setTargetPosition(EXTENDERMIN);
                return false;
            }
        }

        public Action moveDown() {
            initialized = false;
            return new MoveDown();
        }
    }

    public class Intake {
        private DcMotorEx horizontalExtender;

        private IMU imu;
        private DcMotor frontRight;
        private DcMotor backRight;
        private DcMotor frontLeft;
        private DcMotor backLeft;
        private boolean searchColorInit = false;

        public Intake(HardwareMap hardwareMap) {
            horizontalExtender = hardwareMap.get(DcMotorEx.class, "horizontalExtender");
            horizontalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalExtender.setTargetPosition(horizontalExtender.getCurrentPosition());
            horizontalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake/parallel");
            imu = hardwareMap.get(IMU.class, "imu");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        public class ClawServoUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1); //alter as necessary
                return false;
            }
        }

        public Action clawServoUp() {
            return new ClawServoUp();
        }

        public class ClawServoDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0); //alter as necessary
                return false;
            }
        }

        public Action clawServoDown() {
            return new ClawServoDown();
        }


        public class SearchColor implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                float VELOCITYTANGENTIAL = 1000;
                int INCREMENT = 250;
                float rotationSpeed;
                double searchOrigin = 0;
                double armMin = 0;
                double armMax = 0;
                int targetedAngle = 1;
                if (!searchColorInit) {
                    armMin = horizontalExtender.getCurrentPosition() - 3;
                    armMax = armMin - 2000;
                    targetedAngle = 1;
                    searchColorInit = true;
                    searchOrigin = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                }
                while (!(colorDetection(colorDetector).equals("Yellow") || colorDetection(colorDetector).equals("Red"))) {
                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    float directionBetweenAngles = distanceBetweenAngles((float) botHeading, (float) (30 * targetedAngle + searchOrigin));
                    if (horizontalExtender.getCurrentPosition() > -500) { //might need to change values for 2-stage
                        rotationSpeed = 0.9f; //no easing in the beginning
                    } else {
                        rotationSpeed = (VELOCITYTANGENTIAL / (-(arm.getCurrentPosition() + 200))); //rotationSpeed (omega) = Vt/r where R is ARMMAX ~ 3000
                    }
                    int velocityArm = (int) (10 * ((215 * rotationSpeed) / (60f)));
                    while (horizontalExtender.getCurrentPosition() > -500) {
                        intakeArmMovement(false, true, INCREMENT, horizontalExtender, armMax, armMin);
                    }
                    intakeArmMovement(false, true, velocityArm, horizontalExtender, armMax, armMin);
                    rotateTo((30 * targetedAngle) + searchOrigin, rotationSpeed, imu, backLeft, backRight, frontLeft, frontRight);
                    if (Math.abs(directionBetweenAngles) < 4) { //determines if the robot is facing a direction
                        if (targetedAngle == 1) { //if it was turning one way, switch it
                            targetedAngle = -1;
                        } else {
                            targetedAngle = 1;
                        }
                    }
                }
                try {
                    TimeUnit.SECONDS.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                while (!(colorDetection(colorDetector).equals("Yellow") || colorDetection(colorDetector).equals("Red"))) {
                    chassisMovement(0, 0, (-0.3f * targetedAngle), imu, backLeft, backRight, frontLeft, frontRight);
                }
                return false;
            }
        }

        public Action searchColor() {
            searchColorInit = false;
            return new SearchColor();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndSetUp();
        waitForStart();
        postStartSetUp();
        Pose2d initialPose = new Pose2d(-32, -61, Math.toRadians(90));
        Pose2d afterDrop = new Pose2d(-48, -48, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        BucketMovement bucketMovement = new BucketMovement(hardwareMap);
        VerticalExtension verticalExtender = new VerticalExtension(hardwareMap);

        TrajectoryActionBuilder spike1FirstHalf = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-38, -15), 3 * (Math.PI / 2))
                .splineTo(new Vector2d(-38, -24), Math.toRadians(270));

        TrajectoryActionBuilder spike1SecondHalf = drive.actionBuilder(new Pose2d(-38, -24, Math.toRadians(270)))
                .splineTo(new Vector2d(-48, -48), Math.toRadians(225));

        TrajectoryActionBuilder spike2FirstHalf = drive.actionBuilder(afterDrop)
                .splineTo(new Vector2d(-60, -15), 3 * (Math.PI / 2))
                .splineTo(new Vector2d(-60, -24), Math.toRadians(270));

        TrajectoryActionBuilder spike2SecondHalf = drive.actionBuilder(new Pose2d(-60, -24, Math.toRadians(270)))
                .splineTo(new Vector2d(-48, -48), Math.toRadians(225));

        TrajectoryActionBuilder spike3FirstHalf = drive.actionBuilder(afterDrop)
                .splineTo(new Vector2d(-61, -15), Math.toRadians(200))
                .splineTo(new Vector2d(-61, -24), Math.toRadians(200));

        TrajectoryActionBuilder spike3SecondHalf = drive.actionBuilder(new Pose2d(-61, -24, Math.toRadians(200)))
                .splineTo(new Vector2d(-48, -48), Math.toRadians(225));


        Action firstSpikeFirstHalf = spike1FirstHalf.build();
        Action firstSpikeSecondHalf = spike1SecondHalf.build();
        Action secondSpikeFirstHalf = spike2FirstHalf.build();
        Action secondSpikeSecondHalf = spike2SecondHalf.build();
        Action thirdSpikeFirstHalf = spike3FirstHalf.build();
        Action thirdSpikeSecondHalf = spike3SecondHalf.build();

        intakeMotor.setPower(0.8);
        Actions.runBlocking(
                new SequentialAction(
                        firstSpikeFirstHalf,
                        firstSpikeSecondHalf,
                        intake.clawServoUp()
                ));

        intakeMotor.setPower(0);
        Actions.runBlocking(
                new SequentialAction(
                        bucketMovement.bucketUp(),
                        verticalExtender.moveUp(),
                        bucketMovement.bucketDown(),
                        verticalExtender.moveDown(),
                        intake.clawServoDown(),
                        bucketMovement.bucketUp()
                ));

        intakeMotor.setPower(0.8);
        Actions.runBlocking(
                new SequentialAction(
                        secondSpikeFirstHalf,
                        secondSpikeSecondHalf,
                        intake.clawServoUp()
                ));

        intakeMotor.setPower(0);
        Actions.runBlocking(
                new SequentialAction(
                        bucketMovement.bucketUp(),
                        verticalExtender.moveUp(),
                        bucketMovement.bucketDown(),
                        verticalExtender.moveDown(),
                        intake.clawServoDown(),
                        bucketMovement.bucketUp()
                ));

        intakeMotor.setPower(0.8);
        Actions.runBlocking(
                new SequentialAction(
                        thirdSpikeFirstHalf,
                        thirdSpikeSecondHalf,
                        intake.clawServoUp()
                ));

        intakeMotor.setPower(0);
        Actions.runBlocking(
                new SequentialAction(
                        bucketMovement.bucketUp(),
                        verticalExtender.moveUp(),
                        bucketMovement.bucketDown(),
                        verticalExtender.moveDown(),
                        intake.clawServoDown(),
                        bucketMovement.bucketUp()
                ));

    }
}


