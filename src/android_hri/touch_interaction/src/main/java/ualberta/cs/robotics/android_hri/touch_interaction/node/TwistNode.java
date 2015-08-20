package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Twist;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class TwistNode implements NodeMain {

    private String nodeName = "customTwist";
    private Publisher<Twist> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private float[] publish_angular =new float[]{0, 0, 0,};
    private float[] publish_linear =new float[]{0, 0, 0,};

    private float[] subcribe_angular =new float[]{0, 0, 0,};
    private float[] subcribe_linear =new float[]{0, 0, 0,};

    private String subscribeTopic;
    private String publishTopic;

    private boolean alwaysPublish;
    private int maxPublishing = 0;
    private boolean hasReceivedMsg=false;
    private boolean hasPublishedMsg=false;

    public void subscribeTo(String topic){
        isSubscriber=true;
        subscribeTopic=topic;
    }

    public void publishTo(String topic, boolean alwaysPublishing, int maxPublishings){
        isPublisher=true;
        publishTopic=topic;
        alwaysPublish=alwaysPublishing;
        maxPublishing=maxPublishings;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(isPublisher){
            publisher = connectedNode.newPublisher(publishTopic, Twist._TYPE);
            publish(connectedNode);
        }
        if(isSubscriber){
            subscribe(connectedNode);
        }
    }


    private void publish(ConnectedNode connectedNode){
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                if(alwaysPublish)
                    publish();
                else{
                    if(counter>0){
                        publish();
                    }
                    counter--;
                }
                Thread.sleep(10);
            }
        };
        connectedNode.executeCancellableLoop(aLoop);
    }

    private void publish(){
        Twist twist = publisher.newMessage();
        twist.getLinear().setX(publish_linear[1]);
        twist.getLinear().setY(publish_linear[0]);
        twist.getLinear().setZ(publish_linear[2]);
        twist.getAngular().setX(publish_angular[1]);
        twist.getAngular().setY(publish_angular[0]);
        twist.getAngular().setZ(publish_angular[2]);
        publisher.publish(twist);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Twist> subscriber = connectedNode.newSubscriber(subscribeTopic, Twist._TYPE);

        subscriber.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist twist) {
                subcribe_angular[1] = (float) twist.getAngular().getX();
                subcribe_angular[0] = (float) twist.getAngular().getY();
                subcribe_angular[2] = (float) twist.getAngular().getZ();
                subcribe_linear[1] = (float) twist.getLinear().getX();
                subcribe_linear[0] = (float) twist.getLinear().getY();
                subcribe_linear[2] = (float) twist.getLinear().getZ();
                hasReceivedMsg=true;
            }
        });
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

    public float[] getPublish_angular() {
        return publish_angular;
    }

    public void setPublish_angular(float[] publish_angular) {
        this.publish_angular = publish_angular;
    }

    public float[] getPublish_linear() {
        return publish_linear;
    }

    public void setPublish_linear(float[] publish_linear) {
        this.publish_linear = publish_linear;
    }

    public float[] getSubcribe_angular() {
        return subcribe_angular;
    }

    public void setSubcribe_angular(float[] subcribe_angular) {
        this.subcribe_angular = subcribe_angular;
    }

    public float[] getSubcribe_linear() {
        return subcribe_linear;
    }

    public void setSubcribe_linear(float[] subcribe_linear) {
        this.subcribe_linear = subcribe_linear;
    }

    public boolean hasReceivedMsg() {
        return hasReceivedMsg;
    }

    public void setHasReceivedMsg(boolean hasReceivedMsg) {
        this.hasReceivedMsg = hasReceivedMsg;
    }

    public boolean hasPublishedMsg() {
        return hasPublishedMsg;
    }

    public void setHasPublishedMsg(boolean hasPublishedMsg) {
        this.hasPublishedMsg = hasPublishedMsg;
    }

}
