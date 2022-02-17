// region Java Imports
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

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
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
// endregion

class WebcamHelper {

    public String deviceName = "Webcam 1";
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
        deviceName = cameraName;
        cameraName = hardwareName;
        
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
    }

    public void initiate () {

        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(2));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap frame) {
                frame.recycle();
            }
        });

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        openCamera();
        if (camera == null) return;

        startCamera();
        if (cameraCaptureSession == null) return;
        
    }

    /**
     * Get a single bitmap frame from the webcam
     */
    public Bitmap getFrame () {

        boolean attemptingCapture = true;
        Bitmap bmp;

        while (attemptingCapture) {

            bmp = frameQueue.poll();

            if (bmp != null) {
                attemptingCapture = false;
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
                public void onConfigured(CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override
                                    public void onNewFrame(CameraCaptureSession session, CameraCaptureRequest request, CameraFrame cameraFrame) {
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override
                                    public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {}
                                })
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

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    /**
     * Get the index of the highest number in an array
     */
    public int getGreenSegment (int[] sections) {

        int section = 0;
        int section_size = 0;

        for (int i = 0; i < sections.length; ++i) {
            if (sections[i] > section_size) {
                section = i;
                section_size = sections[i];
            } 
        }

        return section;
    }

    /**
     * Save the bitmap as a JPG image
     */
    public boolean saveBitmap (Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        
        try (FileOutputStream outputStream = new FileOutputStream(file)) {
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            return true;
        }
        return false;
    }

    /**
     * Scale down a bitmap by a percent
     */
    public Bitmap scaleBitmap (Bitmap bmp, double imageCompression) {
        int newWidth = frame.getWidth() * imageCompression;
        int newHeight = frame.getHeight() * imageCompression;

        return Bitmap.createScaledBitmap(frame, newWidth, newHeight, false);
    }

    /**
     * TODO
     * Use getPixels once to load entire bitmap
     * rather than getPixel on each pixel
     */
    private int[] getMajorityGreen (Bitmap bmp, int colorThreshold) {

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
