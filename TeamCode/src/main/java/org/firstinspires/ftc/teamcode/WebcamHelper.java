class WebcamHelper {

    private static int captureCounter = 0;

    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    private String deviceName = "Webcam 1";

    public WebcamHelper (String cameraName) {
        deviceName = cameraName;

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, deviceName);

    }

    public void initiate () {
        try {

            openCamera();
            if (camera == null) return;

        } finally {
            closeCamera
        }
        
    }

    private void closeCamera () {
        
    }

    private void openCamera () {

    }

    private void startCamera () {

    }

    public static int getGreenSegment (int[] sections) {

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

    public static boolean saveBitmap (Bitmap bitmap, File captureDirectory) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        
        try (FileOutputStream outputStream = new FileOutputStream(file)) {
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            return true;
        }
        return false;
    }

    public static Bitmap scaleBitmap (Bitmap bmp, double ratio) {
        int newWidth = frame.getWidth() * ratio;
        int newHeight = frame.getHeight() * ratio;

        return Bitmap.createScaledBitmap(frame, newWidth, newHeight, false);
    }

    /**
     * TODO
     * Use getPixels once to load entire bitmap
     * rather than getPixel on each pixel
     */
    private static int[] getMajorityGreen (Bitmap bmp) {
        getMajorityGreen(bmp, 3);
    }

    private static int[] getMajorityGreen (Bitmap bmp, int segments) {

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
