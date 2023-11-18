//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class DistanceSensorCalibration extends LinearOpMode {
//
//    private Rev2mDistanceSensor distanceSensor;
//    private final double TARGET_DISTANCE = 10.0; // Your desired target distance in inches
//    private final double TOLERANCE = 0.2; // Tolerance for acceptable readings
//
//    @Override
//    public void runOpMode() {
//        HardwareMap hardwareMap = hardwareMap;
//        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
//
//            // Adjust sensor settings based on current distance
//            if (currentDistance < (TARGET_DISTANCE - TOLERANCE)) {
//                // If the current distance is too low, increase sensor sensitivity or adjust settings accordingly
//                // Example: distanceSensor.setSensitivity(sensitivityValue);
//                telemetry.addData("Status", "Sensor reading too low");
//            } else if (currentDistance > (TARGET_DISTANCE + TOLERANCE)) {
//                // If the current distance is too high, decrease sensor sensitivity or adjust settings accordingly
//                // Example: distanceSensor.setSensitivity(sensitivityValue);
//                telemetry.addData("Status", "Sensor reading too high");
//            } else {
//                // Sensor is within the acceptable range
//                // You may perform other actions or log that the sensor is calibrated
//                telemetry.addData("Status", "Sensor calibrated");
//            }
//
//            telemetry.addData("Distance", currentDistance);
//            telemetry.update();
//        }
//    }
//}
