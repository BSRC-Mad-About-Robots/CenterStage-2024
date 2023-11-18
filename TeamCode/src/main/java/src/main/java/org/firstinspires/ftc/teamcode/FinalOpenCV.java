package src.main.java.org.firstinspires.ftc.teamcode;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.openftc.easyopencv.OpenCvPipeline;
import java.io.FileOutputStream;
import java.io.File;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;
import java.util.List;
import android.graphics.Bitmap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;
import android.os.Environment;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class FinalOpenCV extends OpenCvPipeline {
    Telemetry telemetry;
    double maxArea;
    int recta=0;
    int rectafinal;
    MatOfPoint largestContour;
    public FinalOpenCV(Telemetry t) {
        telemetry=t;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 35, 105);
        Scalar highHSV = new Scalar(200, 355, 355);
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Rect[] rectangles = {
                new Rect(150, 287, 200, 233), // Adjust the coordinates and dimensions
                new Rect(470, 287, 430, 233), // Adjust the coordinates and dimensions
                new Rect(1000, 287, 240, 233) };
        for (Rect rect : rectangles) {
            Mat regionOfInterest = new Mat(thresh, rect);
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(regionOfInterest, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (!contours.isEmpty()) {
                if(recta==0){
            largestContour = contours.get(0);
            maxArea = Imgproc.contourArea(largestContour);}
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    rectafinal=recta;
                    maxArea = area;
                    largestContour = contour;
                }}}recta++;}
        recta=0;
        Imgproc.rectangle(
                mat,
                new Point(
                        150,
                        287),
                new Point(
                        1240,
                        520),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                mat,
                new Point(
                        150,
                        287),
                new Point(
                        350,
                        520),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                mat,
                new Point(
                        470,
                        287),
                new Point(
                        900,
                        520),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                mat,
                new Point(
                        1000,
                        287),
                new Point(
                        1240,
                        520),
                new Scalar(0, 255, 0), 4);
        if (largestContour != null) {
            Mat output = new Mat(thresh.size(), CvType.CV_8UC1, Scalar.all(0));
            Imgproc.drawContours(output, Arrays.asList(largestContour), -1, new Scalar(255), -1);
            mat.copyTo(input);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

            return mat;
        } else {
            rectafinal=-1;
            return thresh;
        }}
    public int getPostitionCapstone(){
        return rectafinal;
    }}