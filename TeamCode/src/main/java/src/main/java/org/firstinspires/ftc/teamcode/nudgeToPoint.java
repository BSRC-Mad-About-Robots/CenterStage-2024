package src.main.java.org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Gotopoint", group="Linear OpMode")
public class nudgeToPoint extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    double pow=0.3;
    double distanceToTarget;
    boolean entered=false;
    boolean distancereached=false;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initAprilTag();
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        waitForStart();
        sleep(2000); // Wait for initial setup, adjust as needed

        while (opModeIsActive()&&!distancereached) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            telemetry.update();
            if( currentDetections.size()<1){
                StopMotors();
                distancereached=true;
                telemetry.addData("force",distanceToTarget);
                telemetry.update();
                sleep(2000);
                break;
            }
            // Check if AprilTag 9 is detected and adjust movement accordingly
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == 9) {
                    pow=pow*0.9;
                    entered=true;
                    double targetX = 0.0; // Adjust target X coordinate as needed
                    double targetY =10 ; // Adjust target Y coordinate as needed
                    double robotX = detection.ftcPose.x;
                    double robotY = detection.ftcPose.y;

                    double deltaX = targetX - robotX;
                    double deltaY = targetY - robotY;
                    double angleToTarget = Math.atan2(deltaY, deltaX);
                    distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                    // Adjust motor powers based on distance and angle
                    double scaledSpeed = 1; // Adjust maximum speed as needed
                    double vertical = Math.sin(angleToTarget) * scaledSpeed;
                    double horizontal = Math.cos(angleToTarget) * scaledSpeed;

                    GoPoint(robotX, robotY, vertical, horizontal); // Adjust power_factor as needed
                    telemetry.addData("vert",vertical);
                    telemetry.addData("horiz",horizontal);
                    telemetry.addData("angleToTarget",angleToTarget);
                    telemetry.addData("distance",distanceToTarget);
                    telemetry.update();
                    // If close enough to the target, stop the robot
                    if (distanceToTarget < 0.14) { // Adjust threshold as needed
                        StopMotors();
                        distancereached=true;
                        telemetry.addData("stopped",true);
                        telemetry.update();
                        sleep(2000);
                        break;
                    }

                }

            }
            if(entered){
                entered=false;
            }
            else{
                StopMotors();
                distancereached=true;
                break;
            }
        }
    }

    private void StopMotors() {
        FR.setPower(0.0);
        BR.setPower(0.0);
        FL.setPower(0.0);
        BL.setPower(0.0);
    }

    private void GoPoint(double x1, double y1, double vertical, double horizontal) {
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double angle = 0;
        if (horizontal != 0.0) {
            angle = Math.toDegrees(Math.atan(vertical / horizontal));
        } else {
            angle = 135000;
            vertical = 1;
            horizontal = 0;
        }
        if ((angle >= 45 && angle <= 90)) {
            horizontal = 1 / (vertical / horizontal);
            vertical = 1;
        } else if (angle < 45 && angle >= 0) {
            vertical = 1 / (horizontal / vertical);
            horizontal = 1;
        } else if (angle < 0 && angle >= -45) {
            vertical = Math.abs(1 / (horizontal / vertical));
            horizontal = -1;
        } else if (angle < -45 && angle >= -90) {
            horizontal = 1 / (vertical / horizontal);
            vertical = 1;
        }

        FR.setPower((vertical - horizontal) * pow);
        BR.setPower((vertical + horizontal) * pow);
        FL.setPower((vertical + horizontal) * pow);
        BL.setPower((vertical - horizontal) * pow);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }
}
