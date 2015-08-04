package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.view.MotionEvent;
import android.view.View;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class SingleDragGestureDetector {

    private OnSingleDragGestureListener mListener;
    private float sX, sY;

    public SingleDragGestureDetector(OnSingleDragGestureListener listener){
        mListener = listener;
    }

    public boolean onTouchEvent( View view, MotionEvent event){
        switch (event.getAction()) {
            case MotionEvent.ACTION_MOVE:
                float[] touch = new float[]{event.getX(), event.getY()}; //relative to the view. Right,Down increase
                float[] viewCenter = new float[]{view.getWidth() / 2, view.getHeight() / 2};
                float[] touchVector = new float[] {touch[0] - viewCenter[0], touch[1] - viewCenter[1]};

                float x =  touchVector[0]/viewCenter[0];
                float y = -touchVector[1]/viewCenter[1];
                if(x > 1 || x < -1 || y > 1 || y < -1){
                    sX=0;
                    sY=0;
                }else{
                    sX=x;
                    sY=y;
                }
                mListener.OnSingleDrag(this);
                break;
            case MotionEvent.ACTION_UP:
                sX = 0;
                sY = 0;
                mListener.OnSingleDrag(this);
                break;
        }
        return true;
    }

    public static interface OnSingleDragGestureListener {
        public void OnSingleDrag(SingleDragGestureDetector singleDragGestureDetector);
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
}