package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import std_msgs.Bool;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class ConfirmNode implements NodeMain {

    private String nodeName = "customJSlider";
    private Publisher<Bool> publisher = null;
    private boolean confirm = false;

    @Override
    public void onStart(ConnectedNode node) {
        publisher = node.newPublisher("/android/active", Bool._TYPE);  //"/android/joynode/joy"
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                Bool bool = publisher.newMessage();
                bool.setData(confirm);
                publisher.publish(bool);
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

    public boolean isConfirm() {
        return confirm;
    }

    public void setConfirm(boolean confirm) {
        this.confirm = confirm;
    }
}
