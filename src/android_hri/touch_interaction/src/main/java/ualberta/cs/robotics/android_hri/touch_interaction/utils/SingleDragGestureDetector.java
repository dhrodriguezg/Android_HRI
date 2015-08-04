package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.view.MotionEvent;
import android.view.View;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class SingleDragGestureDetector {

    private OnSingleDragGestureListener mListener;
    private float sX, sY;
    private float normalizedX;
    private float normalizedY;
    private boolean release=true;

    public SingleDragGestureDetector(OnSingleDragGestureListener listener){
        mListener = listener;
    }

    public boolean onTouchEvent( View view, MotionEvent event){
        switch (event.getAction()) {
            case MotionEvent.ACTION_MOVE:
                release=false;
                sX=event.getX(); sY=event.getY();
                float[] touch = new float[]{event.getX(), event.getY()}; //relative to the view. Right,Down increase
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
                mListener.OnSingleDrag(this);
                break;
            case MotionEvent.ACTION_UP:
                release=true;
                mListener.OnSingleDrag(this);
                break;
        }
        return true;
    }

    public static interface OnSingleDragGestureListener {
        public void OnSingleDrag(SingleDragGestureDetector singleDragGestureDetector);
    }

    public boolean isRelease(){
        return release;
    }

    public float getX() {
        return sX;
    }

    public void setX(float x) {
        this.sX = x;
    }

    public float getY() {
        return sY;
    }

    public void setY(float y) {
        this.sY = y;
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