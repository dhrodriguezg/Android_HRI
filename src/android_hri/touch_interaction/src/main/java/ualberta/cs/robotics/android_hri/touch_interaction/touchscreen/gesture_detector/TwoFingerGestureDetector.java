package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

/**
 * Created by Diego Rodriguez on 25/07/2015.
 */
public class TwoFingerGestureDetector extends ScaleGestureDetector.SimpleOnScaleGestureListener {

    private static final String TAG = "TwoFingerGesture";

    private OnTwoFingerGestureListener mListener;
    private ScaleGestureDetector mGestureDetector;
    private Activity mActivity;
    private View mView;

    private static final int INVALID_POINTER_ID = -1;
    private static final int DRAGGING_THREASHOLD = 20;
    private static final int ROTATING_THREASHOLD = 10;
    private static final int SCALING_THREASHOLD = 100;

    private static final float MIN_SCALE = 0.1f;//full open 0
    private static final float MAX_SCALE = 5.1f;//full close 3.2
    //calculate grasp ->   (MAX_SCALE-X)*3.2/(MAX_SCALE-MIN_SCALE)
    private boolean detectingGesture =false;
    private float fX, fY, sX, sY;
    private int ptrID1, ptrID2;

    private float mAngle;
    private float mX, mY;
    private float normalizedX;
    private float normalizedY;
    private float mScale = 1.f;
    private float mScaleFocusX = 0.f;
    private float mScaleFocusY = 0.f;

    private float initScale = mScale;
    private float endScale = mScale;
    private long initTime=0;
    private long endTime=0;


    private boolean rotating=true;
    private boolean dragging=true;
    private boolean scaling=true;




    public TwoFingerGestureDetector(Activity activity, OnTwoFingerGestureListener listener){
        mGestureDetector = new ScaleGestureDetector(activity, this);
        mActivity = activity;
        mListener = listener;
        ptrID1 = INVALID_POINTER_ID;
        ptrID2 = INVALID_POINTER_ID;
    }

    /** Low-level events **/

    public boolean onTouchEvent(View view, MotionEvent event){
        mView = view;
        calculateGestures(view,event);
        return mGestureDetector.onTouchEvent(event);
    }

    private boolean calculateGestures(View view, MotionEvent event) {

        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                ptrID1 = event.getPointerId(event.getActionIndex());
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                detectingGesture = true;
                ptrID2 = event.getPointerId(event.getActionIndex());
                sX = event.getX(event.findPointerIndex(ptrID1));
                sY = event.getY(event.findPointerIndex(ptrID1));
                fX = event.getX(event.findPointerIndex(ptrID2));
                fY = event.getY(event.findPointerIndex(ptrID2));
                mListener.onTwoFingerGestureState(detectingGesture);
                Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
                break;
            case MotionEvent.ACTION_MOVE:
                if (ptrID1 != INVALID_POINTER_ID && ptrID2 != INVALID_POINTER_ID) {
                    detectingGesture = true;
                    float nfX, nfY, nsX, nsY, nX, nY;
                    nsX = event.getX(event.findPointerIndex(ptrID1));
                    nsY = event.getY(event.findPointerIndex(ptrID1));
                    nfX = event.getX(event.findPointerIndex(ptrID2));
                    nfY = event.getY(event.findPointerIndex(ptrID2));

                    if(dragging){
                        mX=(nsX+nfX)/2; mY=(nsY+nfY)/2;

                        float[] normalizedXY = normalizedValues(mX,mY,mView);
                        normalizedX = normalizedXY[0];
                        normalizedY = normalizedXY[1];

                        nX=(sX+fX)/2; nY=(sY+fY)/2;
                        if(Math.hypot(mX-nX,mY-nY) > DRAGGING_THREASHOLD){
                            rotating=false;
                            scaling=false;
                            mListener.OnDoubleDrag(mX, mY, normalizedX, normalizedY);
                            Log.d(TAG, String.format("Drag [ %.4f , %.4f ]", mX,mY));
                        }
                    }

                    if(rotating){
                        mAngle = angleBetweenLines(fX, fY, sX, sY, nfX, nfY, nsX, nsY);
                        if(Math.abs(mAngle) > ROTATING_THREASHOLD){
                            scaling=false;
                            dragging=false;
                            mListener.OnRotation(mAngle);
                            Log.d(TAG, String.format("Angle [ %.4f ]", mAngle));
                        }
                    }
                }
                break;
            case MotionEvent.ACTION_UP:
                ptrID1 = INVALID_POINTER_ID;
                detectingGesture = false;
                rotating=true;
                dragging=true;
                scaling=true;
                mListener.onTwoFingerGestureState(detectingGesture);
                Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
                break;
            case MotionEvent.ACTION_POINTER_UP:
                ptrID2 = INVALID_POINTER_ID;
                detectingGesture = false;
                rotating=true;
                dragging=true;
                scaling=true;
                mListener.onTwoFingerGestureState(detectingGesture);
                Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
                break;
            case MotionEvent.ACTION_CANCEL:
                detectingGesture = false;
                ptrID1 = INVALID_POINTER_ID;
                ptrID2 = INVALID_POINTER_ID;
                mListener.onTwoFingerGestureState(detectingGesture);
                Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
                break;
        }
        return true;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector detector){
        if(scaling) {
            initTime = detector.getEventTime();
            initScale = mScale;
            rotating=false;
            dragging=false;
        }
        return true;
    }

    @Override
    public boolean onScale(ScaleGestureDetector detector){
        if(scaling) {
            mScaleFocusX=detector.getFocusX();
            mScaleFocusY=detector.getFocusY();
            mScale *= detector.getScaleFactor();
            mScale = Math.max(MIN_SCALE, Math.min(mScale, MAX_SCALE));
            mListener.OnScale(mScale,mScaleFocusX,mScaleFocusY);
            Log.d(TAG, String.format("Scale [ %.4f %.4f %.4f]", mScale,mScaleFocusX,mScaleFocusY));
        }
        return true;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector detector){
        if(scaling) {
            endTime=detector.getEventTime();
            endScale = mScale;
            if( endTime-initTime < SCALING_THREASHOLD){
                //Fast Pinch
                if(endScale > initScale){
                    mScale =MAX_SCALE;
                }else if(endScale < initScale ){
                    mScale =MIN_SCALE;
                }
                mScaleFocusX=detector.getFocusX();
                mScaleFocusY=detector.getFocusY();
                mListener.OnScale(mScale,mScaleFocusX,mScaleFocusY);
                Log.d(TAG, String.format("Scale [ %.4f %.4f %.4f]", mScale,mScaleFocusX,mScaleFocusY));
            }
        }
    }

    /** Gesture interfaces **/

    public static interface OnTwoFingerGestureListener {
        public void OnDoubleDrag(float mX,float mY,float normalizedX,float normalizedY);
        public void OnRotation(float mAngle);
        public void OnScale(float mScale, float mScaleFocusX, float mScaleFocusY);
        public void onTwoFingerGestureState(boolean detectingGesture);
    }

    /** Normalized values **/

    private float[] normalizedValues(float mX, float mY, View view){
        float[] touch = new float[]{mX,mY}; //relative to the view. Right,Down increase
        float[] viewCenter = new float[]{view.getWidth() / 2, view.getHeight() / 2};
        float[] touchVector = new float[] {touch[0] - viewCenter[0], touch[1] - viewCenter[1]};
        touchVector[0]/=  viewCenter[0];
        touchVector[1]/= -viewCenter[1];
        if(touchVector[0] > 1 || touchVector[0] < -1 || touchVector[1] > 1 || touchVector[1] < -1){
            touchVector[0] = 0;
            touchVector[1] = 0;
        }
        return touchVector;
    }

    private float angleBetweenLines (float fX, float fY, float sX, float sY, float nfX, float nfY, float nsX, float nsY) {
        float angle1 = (float) Math.atan2((fY - sY), (fX - sX));
        float angle2 = (float) Math.atan2((nfY - nsY), (nfX - nsX));

        float angle = ((float) Math.toDegrees(angle1 - angle2)) % 360;
        if (angle < -180.f) angle += 360.0f;
        if (angle > 180.f) angle -= 360.0f;
        return angle;
    }

    /** Getters and Setters **/

    public float getmScale() {
        return mScale;
    }

    public void setmScale(float mScale) {
        this.mScale = mScale;
    }

    public float getmAngle() {
        return mAngle;
    }

    public void setmAngle(float mAngle) {
        this.mAngle = mAngle;
    }

    public float getmX() {
        return mX;
    }

    public void setmX(float mX) {
        this.mX = mX;
    }

    public float getmY() {
        return mY;
    }

    public void setmY(float mY) {
        this.mY = mY;
    }

    public float getNormalizedX() {
        return normalizedX;
    }

    public void setNormalizedX(float normalizedX) {
        this.normalizedX = normalizedX;
    }

    public float getNormalizedY() {
        return normalizedY;
    }

    public void setNormalizedY(float normalizedY) {
        this.normalizedY = normalizedY;
    }
}
