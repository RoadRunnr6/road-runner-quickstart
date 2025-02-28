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



@Autonomous(name = "redAutoLeft", group = "Autonomous")
public class redAutoLeft extends LinearOpMode {

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

    private void intakeArmMovement(boolean in, boolean out, int increment, DcMotor horizontalExtender, int horizontalExtenderMin, int horizontalExtenderMax) {
        int extenderPosition = horizontalExtender.getCurrentPosition();
        if (in) {       // if (DPAD-down) is being pressed and if not yet the min
            extenderPosition += increment;   // Position in
            extenderPosition = Math.min(Math.max(extenderPosition, horizontalExtenderMax), horizontalExtenderMin);  //clamp the values to be between min and max
        } else if (out) {  // if (DPAD-up) is being pressed and if not yet max
            extenderPosition -= increment;   // Position Out
            extenderPosition = Math.min(Math.max(extenderPosition, horizontalExtenderMax), horizontalExtenderMin);  //clamp the values to be between min and max
        }
        horizontalExtender.setTargetPosition(extenderPosition);
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
        return phi > 180 ? phi - 360 : (phi < -180 ? phi + 360 : phi);
    }

    public class BucketMovement {
        private Servo bucketServo;

        public BucketMovement(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        public class BucketBottom implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(0.95); //alter as necessary
                return false;
            }
        }

        public Action bucketBottom() {
            return new BucketBottom();
        }

        public class BucketMiddle implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(0.8); //alter as necessary
                return false;
            }
        }

        public Action bucketMiddle() {
            return new  BucketMiddle();
        }
        public class BucketDrop implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(0.55); //alter as necessary
                return false;
            }
        }
        public Action bucketDrop() {
            return new BucketDrop();
        }
    }

    public class VerticalExtension {
        private DcMotor verticalExtender;
        private boolean initialized = false;
        int verticalExtenderMin;
        int verticalExtenderMax;

        public VerticalExtension(HardwareMap hardwareMap) {
            verticalExtender = hardwareMap.get(DcMotor.class, "verticalExtender");
            verticalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalExtender.setTargetPosition(verticalExtender.getCurrentPosition());
            verticalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalExtender.setPower(2);
        }

        public class MoveUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    verticalExtenderMin = verticalExtender.getCurrentPosition();
                    verticalExtenderMax = verticalExtenderMin - 4300;
                    initialized = true;
                }
                verticalExtender.setTargetPosition(verticalExtenderMax);
                return false;
            }
        }

        public Action moveUp() {
            initialized = false;
            return new MoveUp();
        }

        public class MoveDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                verticalExtender.setTargetPosition(verticalExtenderMin);
                return false;
            }
        }

        public Action moveDown() {
            return new MoveDown();
        }

        public class Sleep300 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(300);
                return false;
            }
        }

        public Action sleep300() {
            return new Sleep300();
        }

        public class Sleep500 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(450);
                return false;
            }
        }

        public Action sleep500() {
            return new Sleep500();
        }

        public class Sleep1000 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(900);
                return false;
            }
        }

        public Action sleep1000() {
            return new Sleep1000();
        }

        public class Sleep800 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(700);
                return false;
            }
        }

        public Action sleep800() {
            return new Sleep800();
        }

        public class Sleep1200 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(1200);
                return false;
            }
        }

        public Action sleep1200() {
            return new Sleep1200();
        }
        public class Sleep200 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(175);
                return false;
            }
        }

        public Action sleep200() {
            return new Sleep200();
        }

        public class Sleep3500 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                redAutoLeft.this.sleep(2250);
                return false;
            }
        }

        public Action sleep3500() {
            return new Sleep3500();
        }
    }

    public class Intake {
        private DcMotorEx horizontalExtender;
        private IMU imu;
        private DcMotor frontRight;
        private DcMotor backRight;
        private DcMotor frontLeft;
        private DcMotor backLeft;
        private ColorSensor colorDetector;
        private Servo clawServo;
        private DcMotor intakeMotor;
        private int horizontalStartingPosition;



        private boolean searchColorInit = false;

        public Intake(HardwareMap hardwareMap) {
            horizontalExtender = hardwareMap.get(DcMotorEx.class, "horizontalExtender");
            horizontalExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalStartingPosition = horizontalExtender.getCurrentPosition();
            horizontalExtender.setTargetPosition(horizontalExtender.getCurrentPosition());
            horizontalExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontalExtender.setPower(1);

            imu = hardwareMap.get(IMU.class, "imu");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake/parallel");
        }

        public class ClawServoUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.3); //alter as necessary
                return false;
            }
        }

        public Action clawServoUp() {
            return new ClawServoUp();
        }

        public class ClawServoDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.85); //alter as necessary
                return false;
            }
        }

        public Action clawServoDown() {
            return new ClawServoDown();
        }

        public class ClawServoMiddle implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.4); //alter as necessary
                return false;
            }
        }

        public Action clawServoMiddle() {
            return new ClawServoMiddle();
        }


        public class IntakeMotorPickup implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(-0.85);
                return false;
            }
        }

        public Action intakeMotorPickUp() {
            return new IntakeMotorPickup();
        }

        public class IntakeMotorPutDown implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(0.85);
                return false;
            }
        }

        public Action intakeMotorPutDown() {
            return new IntakeMotorPutDown();
        }

        public class IntakeMotorOff implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(0);
                return false;
            }
        }

        public class HorizontalExtenderReset implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                horizontalExtender.setTargetPosition(horizontalStartingPosition); //alter as necessary
                return false;
            }
        }

        public Action horizontalExtenderReset() {
            return new HorizontalExtenderReset();
        }

        public Action intakeMotorOff() {
            return new IntakeMotorOff();
        }

        public class SearchColor implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                float VELOCITYTANGENTIAL = 1000;
                int INCREMENT = 250;
                float rotationSpeed;
                double searchOrigin = 0;
                int horizontalExtenderMin = 0;
                int horizontalExtenderMax = 0;
                int targetedAngle = 1;
                if (!searchColorInit) {
                    horizontalExtenderMin = horizontalExtender.getCurrentPosition() - 3;
                    horizontalExtenderMax = horizontalExtenderMin - 2000;
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
                        rotationSpeed = (VELOCITYTANGENTIAL / (-(horizontalExtender.getCurrentPosition() + 200))); //rotationSpeed (omega) = Vt/r where R is ARMMAX ~ 3000
                    }
                    int velocityArm = (int) (10 * ((215 * rotationSpeed) / (60f)));
                    while (horizontalExtender.getCurrentPosition() > -500) {
                        intakeArmMovement(false, true, INCREMENT, horizontalExtender, horizontalExtenderMax, horizontalExtenderMin);
                    }
                    intakeArmMovement(false, true, velocityArm, horizontalExtender, horizontalExtenderMax, horizontalExtenderMin);
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
        waitForStart();
        Pose2d initialPose = new Pose2d(-29, -68, Math.toRadians(90));
        Pose2d afterDrop = new Pose2d(-58, -60, Math.toRadians(50));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        BucketMovement bucketMovement = new BucketMovement(hardwareMap);
        VerticalExtension verticalExtender = new VerticalExtension(hardwareMap);
        int horizontalStartingPosition = intake.horizontalExtender.getCurrentPosition();
        intake.horizontalExtender.setTargetPosition(-100);
        bucketMovement.bucketServo.setPosition(.95);



        TrajectoryActionBuilder spike1MovementFirstThird = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-30, Math.toRadians(185));

        TrajectoryActionBuilder spike1MovementSecondThird = drive.actionBuilder(new Pose2d(-29, -28.9, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .lineToX(-37);

        TrajectoryActionBuilder spike1MovementThirdThird = drive.actionBuilder(new Pose2d(-37, -28.85, Math.toRadians(180)))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-60, Math.toRadians(30))
                .setTangent(Math.toRadians(180))
                .lineToX(-58);

        TrajectoryActionBuilder spike2MovementFirstThird = drive.actionBuilder(afterDrop)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-38, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-30, Math.toRadians(185));

        TrajectoryActionBuilder spike2MovementSecondThird = drive.actionBuilder(new Pose2d(-38, -28, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .lineToX(-48);

        TrajectoryActionBuilder spike2MovementThirdThird = drive.actionBuilder(new Pose2d(-48, -28, Math.toRadians(180)))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-65, Math.toRadians(30))
                .setTangent(Math.toRadians(180))
                .lineToX(-53);

        Action firstSpikeFirstThird = spike1MovementFirstThird.build();
        Action firstSpikeSecondThird = spike1MovementSecondThird.build();
        Action firstSpikeThirdThird = spike1MovementThirdThird.build();

        Action secondSpikeFirstThird = spike2MovementFirstThird.build();
        Action secondSpikeSecondThird = spike2MovementSecondThird.build();
        Action secondSpikeThirdThird = spike2MovementThirdThird.build();

        while (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(


                    firstSpikeFirstThird,
                    intake.clawServoDown(),
                    verticalExtender.sleep1000(),
                    intake.intakeMotorPickUp(),
                    firstSpikeSecondThird,
                    intake.clawServoUp(),
                    verticalExtender.sleep1200(),
                    intake.intakeMotorPutDown(),
                    verticalExtender.sleep1000(),
                    intake.intakeMotorOff(),
                    intake.clawServoMiddle(),
                    bucketMovement.bucketMiddle(),
                    firstSpikeThirdThird,
                    verticalExtender.moveUp(),
                    verticalExtender.sleep3500(),
                    bucketMovement.bucketDrop(),
                    verticalExtender.sleep500(),
                    verticalExtender.moveDown(),
                    bucketMovement.bucketBottom(),
                        //second spike
                    secondSpikeFirstThird,
                    intake.clawServoDown(),
                    verticalExtender.sleep800(),
                    intake.intakeMotorPickUp(),
                    verticalExtender.sleep200(),
                    secondSpikeSecondThird,
                    intake.clawServoUp(),
                    verticalExtender.sleep1200(),
                    intake.intakeMotorPutDown(),
                    verticalExtender.sleep1000(),
                    intake.intakeMotorOff(),
                    intake.clawServoMiddle(),
                    intake.horizontalExtenderReset(),
                    bucketMovement.bucketMiddle(),
                    secondSpikeThirdThird,
                    verticalExtender.moveUp(),
                    verticalExtender.sleep3500(),
                    bucketMovement.bucketDrop(),
                    verticalExtender.sleep500(),
                    verticalExtender.moveDown(),
                    bucketMovement.bucketBottom(),
                        verticalExtender.sleep3500(),
                        verticalExtender.sleep3500()





                ));
        }
    }
}