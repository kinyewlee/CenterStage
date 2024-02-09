package org.firstinspires.ftc.team15091;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class YellowProcessor implements VisionProcessor {
    private static final Scalar minValues = new Scalar(25, 128, 128);
    private static final Scalar maxValues = new Scalar(35, 255, 255);

    private static final Paint greenPaint = new Paint();

    public double pixelArea = 0;
    public double pixelX;
    public double pixelY;

    private static Rect boundingRect;

    public void init(int width, int height, CameraCalibration calibration) {
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.STROKE);
        greenPaint.setStrokeWidth(2);
        greenPaint.setTextSize(50);
    }

    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat temp = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor(frame, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2HSV);

        // remove noise
        Imgproc.blur(temp, temp, new Size(7, 7));
        Core.inRange(temp, minValues, maxValues, temp);

        Imgproc.findContours(temp, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        pixelArea = 0;
        pixelX = 0;
        pixelY = 0;
        for (int i = 0; i < contours.size(); i++) {
            Moments moments = Imgproc.moments(contours.get(i));
            if (moments.m00 > pixelArea) {
                pixelArea = moments.m00;
                pixelX = moments.m10 / moments.m00;
                pixelY = moments.m01 / moments.m00;
                boundingRect = Imgproc.boundingRect(contours.get(i));
            }
        }

        return null;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (pixelArea > 0) {
            int left = Math.round(boundingRect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(boundingRect.y * scaleBmpPxToCanvasPx);
            canvas.drawRect(
                    left,
                    top,
                    left + Math.round(boundingRect.width * scaleBmpPxToCanvasPx),
                    top + Math.round(boundingRect.height * scaleBmpPxToCanvasPx),
                    greenPaint
            );
            canvas.drawLine(
                    (float)(pixelX * scaleBmpPxToCanvasPx - 10),
                    (float)(pixelY * scaleBmpPxToCanvasPx),
                    (float)(pixelX * scaleBmpPxToCanvasPx + 10),
                    (float)(pixelY * scaleBmpPxToCanvasPx),
                    greenPaint
            );
            canvas.drawLine(
                    (float)(pixelX * scaleBmpPxToCanvasPx),
                    (float)(pixelY * scaleBmpPxToCanvasPx - 10),
                    (float)(pixelX * scaleBmpPxToCanvasPx),
                    (float)(pixelY * scaleBmpPxToCanvasPx + 10),
                    greenPaint
            );
        }
    }
}
