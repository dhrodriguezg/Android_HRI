package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.app.Activity;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by DarkNeoBahamut on 25/07/2015.
 */
public class DoubleTapGestureDetector extends GestureDetector.SimpleOnGestureListener {

    private OnDoubleTapGestureListener mListener;
    private GestureDetector mGestureDetector;
    private Activity mActivity;
    private View mView;
    private float mX;
    private float mY;

    public DoubleTapGestureDetector(Activity activity, OnDoubleTapGestureListener listener){
        mGestureDetector = new GestureDetector(activity, this);
        mActivity = activity;
        mListener = listener;
    }

    public boolean onTouchEvent( View view, MotionEvent event){
        mView = view;
        return mGestureDetector.onTouchEvent(event);
    }

    @Override
    public boolean onDoubleTap(MotionEvent event) {
        float[] touch = new float[]{event.getX(), event.getY()}; //relative to the view. Right,Down increase
        float[] viewCenter = new float[]{mView.getWidth() / 2, mView.getHeight() / 2};
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
        mListener.OnDoubleTap(this);
        return true;
    }

    public static interface OnDoubleTapGestureListener {
        public void OnDoubleTap(DoubleTapGestureDetector doubleTapGestureDetector);
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
