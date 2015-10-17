package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.content.Context;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.util.AttributeSet;
import android.view.Gravity;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.util.Vector;

import ualberta.cs.robotics.android_hri.touch_interaction.R;

/**
 * Created by dhrodriguezg on 10/15/15.
 */
public class ScrollerView extends RelativeLayout{

    private RelativeLayout mainLayout;
    private ScrollView scrollView;
    private LinearLayout viewContainer;
    private Vector<TextView> vectorText;
    private TextView activeView;
    private final int MAX_VISIBLE_ITEMS=5;//odd number
    private final int MAX_ITEMS=10;
    private final float MAX_FONT_SIZE = 20;
    private final float MIN_FONT_SIZE = 8;
    private boolean firstRun;
    private int BACKGROUND = Color.LTGRAY;
    private int SELECTED = Color.RED;
    private boolean revertDirection;

    //gray 193x2
    //black 0x3 191
    //blue 0 162 255

    private float minValue;
    private float maxValue;


    private ImageView top;
    private ImageView bottom;
    private ImageView selection;

    public ScrollerView(Context context) {
        super(context);
        initScroller(context);
    }

    public ScrollerView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initScroller(context);
    }

    public ScrollerView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        initScroller(context);
    }

    private void initScroller(Context context) {

        /** Init Layouts**/
        firstRun = true;
        revertDirection=false;
        mainLayout = this;
        scrollView = new ScrollView(context);
        viewContainer = new LinearLayout(context);
        viewContainer.setOrientation(LinearLayout.VERTICAL);
        top = new ImageView(context);
        bottom = new ImageView(context);
        selection = new ImageView(context);

        mainLayout.addView(scrollView);
        scrollView.addView(viewContainer);
        mainLayout.addView(selection);
        mainLayout.addView(bottom);
        mainLayout.addView(top);

        RelativeLayout.LayoutParams scrollParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT,RelativeLayout.LayoutParams.MATCH_PARENT);
        scrollView.setLayoutParams(scrollParams);
        scrollView.setVerticalScrollBarEnabled(false);
        scrollView.setHorizontalScrollBarEnabled(false);

        FrameLayout.LayoutParams containerParams = new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT);
        viewContainer.setLayoutParams(containerParams);
        viewContainer.setBackgroundColor(Color.DKGRAY);
        viewContainer.setPadding(10, 0, 10, 0);

        RelativeLayout.LayoutParams selectionParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
        selectionParams.addRule(RelativeLayout.CENTER_IN_PARENT);
        selection.setLayoutParams(selectionParams);
        selection.setImageBitmap(BitmapFactory.decodeResource(getResources(), R.drawable.scroller_selection));
        selection.setAlpha(0.7f);
        selection.setScaleType(ImageView.ScaleType.FIT_XY);

        RelativeLayout.LayoutParams bottomParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
        bottomParams.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
        bottomParams.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
        bottom.setLayoutParams(bottomParams);
        bottom.setImageBitmap(BitmapFactory.decodeResource(getResources(), R.drawable.scroller_bottom));
        bottom.setScaleType(ImageView.ScaleType.FIT_XY);

        RelativeLayout.LayoutParams topParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
        topParams.addRule(RelativeLayout.ALIGN_PARENT_TOP);
        topParams.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
        top.setLayoutParams(topParams);
        top.setImageBitmap(BitmapFactory.decodeResource(getResources(), R.drawable.scroller_top));
        top.setScaleType(ImageView.ScaleType.FIT_XY);

        /** Populate container**/
        vectorText= new Vector<>();
        for(int n=0;n<MAX_ITEMS + MAX_VISIBLE_ITEMS-1;n++){
            TextView tv = new TextView(context);
            tv.setBackgroundColor(BACKGROUND);
            tv.setGravity(Gravity.CENTER);
            tv.setText("" + n);
            vectorText.add(tv);
            viewContainer.addView(tv);
        }
        activeView=vectorText.elementAt(MAX_VISIBLE_ITEMS/2);
    }

    public void reverseDirection(){
        firstRun=false;
        revertDirection=!revertDirection;
    }

    public void resizeContainer(){
        int height = scrollView.getHeight()/MAX_VISIBLE_ITEMS;
        top.getLayoutParams().height=height*3/2;
        bottom.getLayoutParams().height=height*3/2;
        selection.getLayoutParams().height=height;

        for(int i = 0; i < vectorText.size(); i++){
            TextView tv;
            if(revertDirection)
                tv = vectorText.elementAt(vectorText.size()-i-1);
            else
                tv = vectorText.elementAt(i);

            tv.setHeight(height);
            float value = (maxValue-minValue)*(float)(i-MAX_VISIBLE_ITEMS/2)/(float)(MAX_ITEMS-1) + minValue;
            if(i-1 < MAX_VISIBLE_ITEMS/2)
                value=minValue;
            if(i > vectorText.size()-MAX_VISIBLE_ITEMS/2-1)
                value=maxValue;

            tv.setText(String.format("%.2f", value));
            tv.setTextSize(14);
            //float fontSize=(MAX_FONT_SIZE-MIN_FONT_SIZE)*value/(maxValue-minValue)+MIN_FONT_SIZE;
            //tv.setTextSize(fontSize);
        }
        firstRun=false;
    }

    public float computeSelection(){
        if(firstRun)
            resizeContainer();

        activeView.setBackgroundColor(BACKGROUND);
        int index = scrollView.getScrollY()/(viewContainer.getHeight()/vectorText.size());
        activeView=vectorText.elementAt(index+MAX_VISIBLE_ITEMS/2);
        activeView.setBackgroundColor(BACKGROUND);
        /*activeView.setTextSize(MAX_FONT_SIZE);
        for(int n=0; n<MAX_VISIBLE_ITEMS/2; n++){
            vectorText.elementAt(index+n).setTextSize((MAX_FONT_SIZE-MIN_FONT_SIZE)*(float)n/((float)MAX_VISIBLE_ITEMS/2) + MIN_FONT_SIZE);
            vectorText.elementAt(index+MAX_VISIBLE_ITEMS-1-n).setTextSize((MAX_FONT_SIZE-MIN_FONT_SIZE)*(float)n/((float)MAX_VISIBLE_ITEMS/2) + MIN_FONT_SIZE);
        }*/
        return Float.parseFloat(activeView.getText().toString());

    }

    public float getMinValue() {
        return minValue;
    }

    public void setMinValue(float minValue) {
        this.minValue = minValue;
    }

    public float getMaxValue() {
        return maxValue;
    }

    public void setMaxValue(float maxValue) {
        this.maxValue = maxValue;
    }
}
