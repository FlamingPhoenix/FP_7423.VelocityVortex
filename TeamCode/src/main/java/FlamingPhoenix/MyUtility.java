package FlamingPhoenix;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by HwaA1 on 12/26/2016.
 */

public class MyUtility {

    public static int getImageAngle(VuforiaTrackable imageObject)
    {
        VuforiaTrackableDefaultListener image = (VuforiaTrackableDefaultListener) imageObject.getListener();
        OpenGLMatrix pos = image.getPose();

        if (pos != null) {
            VectorF c2 = pos.getColumn(2);
            float c2x = c2.get(0);
            float c2z = c2.get(2);
            float zAngle = (float) Math.toDegrees(Math.atan2(c2z, c2x));

            return (int) zAngle;
        }

        return -1;
    }
}
