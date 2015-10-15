package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

/**
 * Created by Diego Rodriguez on 25/07/2015.
 */
public class StatelessGestureDetector{

    private static final String TAG = "StatelessGesture";

    private Activity mActivity;
    private View mView;

    private static final int INVALID_POINTER_ID = -1;
    private static final float MAX_GRASP = 2.f;
    private static final float MIN_GRASP = .1f;
    private static float MAX_DISTANCE=500;
    private static float MIN_DISTANCE=20;

    private boolean detectingGesture =false;

    private float fX, fY, sX, sY;
    private int ptrID1, ptrID2;
    private float mX, mY;
    private float initDistance;
    private float initAngle;
    private float initPosX;
    private float initPosY;

    private float currDistance;
    private float currAngle;
    private float currPosX;
    private float currPosY;

    public StatelessGestureDetector(Activity activity){
        mActivity = activity;
        ptrID1 = INVALID_POINTER_ID;
        ptrID2 = INVALID_POINTER_ID;
    }

    /** Low-level events **/

    public boolean onTouchEvent(View view, MotionEvent event){
        mView = view;
        calculateGestures(view,event);
        return true;
    }

    private void calculateGestures(View view, MotionEvent event) {

        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                ptrID1 = event.getPointerId(event.getActionIndex());
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                detectingGesture = true;
                if(ptrID2 != INVALID_POINTER_ID)
                    return;
                ptrID2 = event.getPointerId(event.getActionIndex());
                sX = event.getX(event.findPointerIndex(ptrID1));
                sY = event.getY(event.findPointerIndex(ptrID1));
                fX = event.getX(event.findPointerIndex(ptrID2));
                fY = event.getY(event.findPointerIndex(ptrID2));

                initPosX = (sX+fX)/2f;
                initPosY = (sY+fY)/2f;
                initDistance = (float) Math.sqrt(Math.pow(sX - fX, 2) + Math.pow(sY - fY, 2));
                initAngle = getAngle(fX, fY, sX, sY);

                break;
            case MotionEvent.ACTION_MOVE:
                if (ptrID1 != INVALID_POINTER_ID && ptrID2 != INVALID_POINTER_ID) {
                    detectingGesture = true;

                    float nfX, nfY, nsX, nsY;
                    nsX = event.getX(event.findPointerIndex(ptrID1));
                    nsY = event.getY(event.findPointerIndex(ptrID1));
                    nfX = event.getX(event.findPointerIndex(ptrID2));
                    nfY = event.getY(event.findPointerIndex(ptrID2));

                    //if(dragging){
                    currPosX=(nsX+nfX)/2f; currPosY=(nsY+nfY)/2f;
                    Log.d(TAG, String.format("Drag [ %.4f , %.4f ]", currPosX,currPosY));

                    //if(rotating){
                    //currAngle = getAngleBetweenLines(fX, fY, sX, sY, nfX, nfY, nsX, nsY);
                    currAngle = getAngle(nfX, nfY, nsX, nsY);
                    Log.d(TAG, String.format("Angle [ %.4f ]", currAngle));

                    //if(scale)
                    currDistance = (float) Math.sqrt(Math.pow(sX - fX, 2) + Math.pow(sY - fY, 2));
                    float normalizedDistance = (MAX_DISTANCE-currDistance)/(MAX_DISTANCE-MIN_DISTANCE);
                    normalizedDistance = Math.max(0,Math.min(normalizedDistance,1));

                    float grasp = (1-normalizedDistance)*(MAX_GRASP-MIN_GRASP)+MIN_GRASP;
                    Log.d(TAG, String.format("Grasp [ %.4f ]", currAngle));
                }
                break;
            case MotionEvent.ACTION_UP:
                ptrID1 = INVALID_POINTER_ID;
                detectingGesture = false;
                break;
            case MotionEvent.ACTION_POINTER_UP:
                ptrID2 = INVALID_POINTER_ID;
                detectingGesture = false;
                break;
            case MotionEvent.ACTION_CANCEL:
                ptrID1 = INVALID_POINTER_ID;
                ptrID2 = INVALID_POINTER_ID;
                detectingGesture = false;
                break;
        }

    }

    private float getAngleBetweenLines (float fX, float fY, float sX, float sY, float nfX, float nfY, float nsX, float nsY) {
        float angle1 = (float) Math.atan2((fY - sY), (fX - sX));
        float angle2 = (float) Math.atan2((nfY - nsY), (nfX - nsX));

        float angle = ((float) Math.toDegrees(angle1 - angle2)) % 360;
        if (angle < -180.f) angle += 360.0f;
        if (angle > 180.f) angle -= 360.0f;
        return angle;
    }

    private float getAngle (float fX, float fY, float sX, float sY) {
        float angle = ((float) Math.toDegrees(Math.atan2((fY - sY), (fX - sX))) ) % 360f;
        if (angle < -180.f) angle += 360.0f;
        if (angle > 180.f) angle -= 360.0f;
        return angle;
    }

}
