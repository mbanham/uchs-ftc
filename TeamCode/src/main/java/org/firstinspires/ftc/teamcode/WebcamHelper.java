// region Java Imports
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
// endregion

class WebcamHelper {

    public String deviceName;
    public int segments = 3;

    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    private static int captureCounter = 0;
    public File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    private EvictingBlockingQueue<Bitmap> frameQueue;
    private Handler callbackHandler;

    public WebcamHelper (String cameraName, WebcamName hardwareName) {

        this.deviceName = cameraName;
        this.cameraName = hardwareName;

        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
    }

    /**
     * Initialize the camera
     */
    public void initiate () {

        // How many frames to store in memory before discarding
        int capacity = 2;

        // Discard excess frames from memory
        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(capacity));
        frameQueue.setEvictAction(Bitmap::recycle);

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        openCamera();
        if (camera == null) return;

        startCamera();
        if (cameraCaptureSession == null) return;

    }

    /**
     * Get a single bitmap frame from the webcam
     * @return returns a bitmap containing the taken image
     */
    public Bitmap getFrame () {

        Bitmap bmp;

        while (true) {

            bmp = frameQueue.poll();

            if (bmp != null) {
                break;
            }
        }

        return bmp;
    }

    /**
     * Stop and turn off the camera
     */
    public void stopCamera () {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }

        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    /**
     * Turns on the camera
     */
    public void openCamera () {
        if (camera != null) return;

        // Timeout camera if it takes longer than 10 seconds
        Deadline deadline = new Deadline(10, TimeUnit.SECONDS);

        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
    }

    /**
     * Start accepting frames from the camera
     */
    private void startCamera () {
        if (cameraCaptureSession != null) return;

        final int imageFormat = ImageFormat.YUY2;

        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();

        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                (session1, request, cameraFrame) -> {
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                },
                                Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) -> {})
                        );
                        synchronizer.finish(session);
                    } catch (CameraException | RuntimeException e) {
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException | RuntimeException e) {
            synchronizer.finish(null);
        }

        // Wait for all the asynchrony to complete
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        cameraCaptureSession = synchronizer.getValue();
    }

    /**
     * Get the index of the highest number in an array
     * @param  sections  array of integers
     * @return           index of the highest number in the array
     */
    public int highestInArray (int[] sections) {

        int index = 0;

        for (int i = 0; i < sections.length; i++) {
            index = sections[i] > sections[index] ? i : index;
        }

        return index;
    }

    /**
     * Save the bitmap as a JPG image
     * @param  bmp  bitmap to save
     * @return      boolean value to determine if the file was successfully saved
     */
    public boolean saveBitmap (Bitmap bmp) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));

        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            bmp.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            return true;
        } catch(Exception ignored) {}

        return false;
    }

    /**
     * Scale down a bitmap by a percent
     */
    public Bitmap scaleBitmap (Bitmap bmp, double imageCompression) {
        int newWidth = (int) (bmp.getWidth() * imageCompression);
        int newHeight = (int) (bmp.getHeight() * imageCompression);

        return Bitmap.createScaledBitmap(bmp, newWidth, newHeight, false);
    }

    /**
     * TODO
     * Use getPixels once to load entire bitmap
     * rather than getPixel on each pixel
     */
    public int[] getMajorityGreen (Bitmap bmp, int colorThreshold) {

        // Get the image dimensions
        int imageHeight = bmp.getHeight();
        int imageWidth = bmp.getWidth();

        // Divide the image into segments
        int segmentLength = imageWidth / segments;

        // Array containing the total number of green pixels in a segment
        int[] averages = new int[segments];

        for (int x = 0; x < imageWidth; x++) {
            for (int y = 0; y < imageHeight; y++) {

                // Get pixel at x, y coordinate
                int pixel = bmp.getPixel(x, y);

                // Get RGB values from hex
                int r = Color.red(pixel) + colorThreshold;
                int g = Color.green(pixel);
                int b = Color.blue(pixel) + colorThreshold;

                // Get index in array
                int index = x / segmentLength;

                // If pixel is green, add to array
                if (index >= segments) {
                    // Fix to ignore pixels which are somehow outside the segment
                } else if (g > r && g > b) {
                    averages[index] += 1;
                    bmp.setPixel(x, y, Color.GREEN);
                } else if (r > g && r > b) {
                    bmp.setPixel(x, y, Color.RED);
                } else if (b > g && b > r) {
                    bmp.setPixel(x, y, Color.BLUE);
                }
            }
        }

        return averages;
    }
}
