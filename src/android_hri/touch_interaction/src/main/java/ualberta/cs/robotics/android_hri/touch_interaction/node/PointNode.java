package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Point;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class PointNode implements NodeMain {

    private String nodeName = "customPoint";
    private Publisher<Point> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private float[] publish_point = new float[]{0, 0, 0};
    private float[] subcribe_point = new float[]{0, 0, 0};

    private String subscribeTopic;
    private String publishTopic;

    private boolean alwaysPublish;
    private int maxPublishing = 0;
    private long publishFreq = 10;

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
            publisher = connectedNode.newPublisher(publishTopic, Point._TYPE);
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
                Thread.sleep(publishFreq);
            }
        };
        connectedNode.executeCancellableLoop(aLoop);
    }

    private void publish(){
        Point point = publisher.newMessage();
        point.setX(publish_point[0]);
        point.setY(publish_point[1]);
        point.setZ(publish_point[2]);
        publisher.publish(point);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Point> subscriber = connectedNode.newSubscriber(subscribeTopic, Point._TYPE);

        subscriber.addMessageListener(new MessageListener<Point>() {
            @Override
            public void onNewMessage(Point point) {
                subcribe_point[0]=(float)point.getX();
                subcribe_point[1]=(float)point.getY();
                subcribe_point[2]=(float)point.getZ();
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

    public float[] getPublish_point() {
        return publish_point;
    }

    public void setPublish_point(float[] publish_point) {
        this.publish_point = publish_point;
    }

    public float[] getSubcribe_point() {
        return subcribe_point;
    }

    public void setSubcribe_point(float[] subcribe_point) {
        this.subcribe_point = subcribe_point;
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

    public long getPublishFreq() {
        return publishFreq;
    }

    public void setPublishFreq(long publishFreq) {
        this.publishFreq = publishFreq;
    }
}
