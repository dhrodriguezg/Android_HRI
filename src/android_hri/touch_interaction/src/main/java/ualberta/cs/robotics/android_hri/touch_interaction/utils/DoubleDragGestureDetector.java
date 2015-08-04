package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.view.MotionEvent;
import android.view.View;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class DoubleDragGestureDetector {
    private static final int INVALID_POINTER_ID = -1;
    private float fX, fY, sX, sY;
    private int ptrID1, ptrID2;
    private float mAngle;
    private float mX, mY;
    private float normalizedX;
    private float normalizedY;
    private boolean release=true;

    private OnDragGestureListener mListener;

    public float getAngle() {
        return mAngle;
    }

    public DoubleDragGestureDetector(OnDragGestureListener listener){
        mListener = listener;
        ptrID1 = INVALID_POINTER_ID;
        ptrID2 = INVALID_POINTER_ID;
    }

    public boolean onTouchEvent(View view, MotionEvent event){
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                ptrID1 = event.getPointerId(event.getActionIndex());
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                ptrID2 = event.getPointerId(event.getActionIndex());
                sX = event.getX(event.findPointerIndex(ptrID1));
                sY = event.getY(event.findPointerIndex(ptrID1));
                fX = event.getX(event.findPointerIndex(ptrID2));
                fY = event.getY(event.findPointerIndex(ptrID2));
                break;
            case MotionEvent.ACTION_MOVE:
                if(ptrID1 != INVALID_POINTER_ID && ptrID2 != INVALID_POINTER_ID){
                    release=false;
                    float nfX, nfY, nsX, nsY;
                    nsX = event.getX(event.findPointerIndex(ptrID1));
                    nsY = event.getY(event.findPointerIndex(ptrID1));
                    nfX = event.getX(event.findPointerIndex(ptrID2));
                    nfY = event.getY(event.findPointerIndex(ptrID2));

                    mX=(nsX+nfX)/2; mY=(nsY+nfY)/2;
                    float[] touch = new float[]{mX,mY}; //relative to the view. Right,Down increase
                    float[] viewCenter = new float[]{view.getWidth() / 2, view.getHeight() / 2};
                    float[] touchVector = new float[] {touch[0] - viewCenter[0], touch[1] - viewCenter[1]};
                    float x =  touchVector[0]/viewCenter[0];
                    float y = -touchVector[1]/viewCenter[1];
                    if(x > 1 || x < -1 || y > 1 || y < -1){
                        normalizedX=0;
                        normalizedY=0;
                    }else{
                        normalizedX=x;
                        normalizedY=y;
                    }
                    mListener.OnDoubleDrag(this);
                }
                break;
            case MotionEvent.ACTION_UP:
                release=true;
                ptrID1 = INVALID_POINTER_ID;
                mListener.OnDoubleDrag(this);
                break;
            case MotionEvent.ACTION_POINTER_UP:
                release=true;
                ptrID2 = INVALID_POINTER_ID;
                mListener.OnDoubleDrag(this);
                break;
            case MotionEvent.ACTION_CANCEL:
                ptrID1 = INVALID_POINTER_ID;
                ptrID2 = INVALID_POINTER_ID;
                mListener.OnDoubleDrag(this);
                break;
        }
        return true;
    }

    public static interface OnDragGestureListener {
        public void OnDoubleDrag(DoubleDragGestureDetector dragDetector);
    }

    public boolean isRelease(){
        return release;
    }

    public float getX() {
        return mX;
    }

    public void setX(float x) {
        this.mX = x;
    }

    public float getY() {
        return mY;
    }

    public void setY(float y) {
        this.mY = y;
    }

    public float getNormalizedY() {
        return normalizedY;
    }

    public void setNormalizedY(float normalizedY) {
        this.normalizedY = normalizedY;
    }

    public float getNormalizedX() {
        return normalizedX;
    }

    public void setNormalizedX(float normalizedX) {
        this.normalizedX = normalizedX;
    }
}