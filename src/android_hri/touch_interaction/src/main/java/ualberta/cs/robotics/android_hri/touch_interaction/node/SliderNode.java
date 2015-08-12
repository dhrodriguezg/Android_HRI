package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import std_msgs.Float32;
/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class SliderNode implements NodeMain {

    private String nodeName = "customJSlider";
    private Publisher<Float32> publisher = null;
    private float sliderValue=0;

    @Override
    public void onStart(ConnectedNode node) {
        publisher = node.newPublisher("/android/slider", Float32._TYPE);  //"/android/joynode/joy"
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                Float32 float32 = publisher.newMessage();
                float32.setData(sliderValue);
                publisher.publish(float32);
                Thread.sleep(10);
            }
        };
        node.executeCancellableLoop(aLoop);

    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    public float getSliderValue() {
        return sliderValue;
    }

    public void setSliderValue(float sliderValue) {
        this.sliderValue = sliderValue;
    }
}
