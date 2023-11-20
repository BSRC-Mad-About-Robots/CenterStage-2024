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
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    double x=100;
    double y=100;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public void GoPoint(int x1,int y1,double vertical,double horizontal,double pow){


        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);


        double angle=0;
        if(horizontal!=0.0)
        {angle=Math.toDegrees(Math.atan(vertical/horizontal));}
        else{
            angle=135000;
            vertical=1;
            horizontal=0;
        }
        telemetry.addData("kkk",angle);
        telemetry.update();
        if((angle>=45&&angle<=90) )
        {
            telemetry.addData("1","true");
            horizontal=1/(vertical/horizontal);
            vertical=1;
        }
        else if(angle<45&&angle>=0){
            telemetry.addData("2","true");
            telemetry.addData("2a",angle);
            vertical=1/(horizontal/vertical);
            horizontal=1;
        }
        else if(angle<0 && angle>=-45)
        {
            telemetry.addData("3",true);
            vertical=Math.abs(1/(horizontal/vertical));
            horizontal=-1;

        }
        else if(angle<-45 && angle>=-90){
            telemetry.addData("4",true);
            horizontal=1/(vertical/horizontal);
            vertical=1;
        }
        telemetry.addData("vertical",vertical);
        telemetry.addData("horizontal",horizontal);
        telemetry.update();
        FR.setPower((vertical - horizontal)*pow);
        BR.setPower((vertical + horizontal)*pow);
        FL.setPower((vertical + horizontal)*pow);
        BL.setPower((vertical - horizontal)*pow);
    }

    @Override
    public void runOpMode(){
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        waitForStart();
sleep(2000);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        while((x<=-0.2|| x>=2)&&y>7){


        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null&&detection.id==9) {
                telemetry.addLine("entered");
                telemetry.update();
                x=detection.ftcPose.x;
                y=detection.ftcPose.y;
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                GoPoint(0,0,y-7,x,0.3);
                if((x>=-0.2&& x<=2)&&y>7)
                {
                    FR.setPower (0.0);
                    BR.setPower (0.0);
                    FL.setPower (0.0);
                    BL.setPower (0.0);
                }
//                sleep(50);

            } else {
                telemetry.addLine("error");
            }
            telemetry.update();
        } }  // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

//        sleep(1000);
        FR.setPower (0.0);
        BR.setPower (0.0);
        FL.setPower (0.0);
        BL.setPower (0.0);
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }
}