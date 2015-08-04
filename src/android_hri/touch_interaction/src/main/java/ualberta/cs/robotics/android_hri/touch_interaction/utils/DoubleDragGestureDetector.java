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
                    float nfX, nfY, nsX, nsY;
                    nsX = event.getX(event.findPointerIndex(ptrID1));
                    nsY = event.getY(event.findPointerIndex(ptrID1));
                    nfX = event.getX(event.findPointerIndex(ptrID2));
                    nfY = event.getY(event.findPointerIndex(ptrID2));

                    float[] touch = new float[]{(nsX+nfX)/2, (nsY+nfY)/2}; //relative to the view. Right,Down increase
                    float[] viewCenter = new float[]{view.getWidth() / 2, view.getHeight() / 2};
                    float[] touchVector = new float[] {touch[0] - viewCenter[0], touch[1] - viewCenter[1]};
                    float x =  touchVector[0]/viewCenter[0];
                    float y = -touchVector[1]/viewCenter[1];
                    if(x > 1 || x < -1 || y > 1 || y < -1){
                        mX=0;
                        mY=0;
                    }else{
                        mX=x;
                        mY=y;
                    }
                    mListener.OnDoubleDrag(this);
                }
                break;
            case MotionEvent.ACTION_UP:
                ptrID1 = INVALID_POINTER_ID;
                break;
            case MotionEvent.ACTION_POINTER_UP:
                ptrID2 = INVALID_POINTER_ID;
                break;
            case MotionEvent.ACTION_CANCEL:
                ptrID1 = INVALID_POINTER_ID;
                ptrID2 = INVALID_POINTER_ID;
                break;
        }
        return true;
    }

    public static interface OnDragGestureListener {
        public void OnDoubleDrag(DoubleDragGestureDetector dragDetector);
    }

    public float getX() {
        return mX;
    }

    public void setX(float x) {
        this.mX = mX;
    }

    public float getY() {
        return mY;
    }

    public void setY(float y) {
        this.mY = mY;
    }
}