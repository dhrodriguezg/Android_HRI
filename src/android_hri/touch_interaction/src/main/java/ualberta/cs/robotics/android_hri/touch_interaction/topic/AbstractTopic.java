package ualberta.cs.robotics.android_hri.touch_interaction.topic;

import org.ros.node.ConnectedNode;

/**
 * Created by dhrodriguezg on 9/28/15.
 */
public abstract class AbstractTopic {

    protected int counter = 0;
    protected boolean isSubscriber;
    protected boolean isPublisher;

    protected String subscriberTopic;
    protected String publisherTopic;

    protected boolean alwaysPublish;
    protected long publishFreq = 10;
    protected int maxPublishing = 0;

    protected boolean hasReceivedMsg=false;
    protected boolean hasPublishedMsg=false;

    public void subscribeTo(String topic){
        isSubscriber=true;
        subscriberTopic =topic;
    }

    public void publishTo(String topic, boolean alwaysPublishing, int maxPublishings){
        isPublisher=true;
        publisherTopic = topic;
        alwaysPublish=alwaysPublishing;
        maxPublishing=maxPublishings;
    }

    public void onStart(ConnectedNode connectedNode) {
        if(isPublisher)
            setupPublisher(connectedNode);
        if(isSubscriber)
            setupSubscriber(connectedNode);
    }

    public void publishNow(){
        counter=maxPublishing;
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

    public long getPublishingFreq() {
        return publishFreq;
    }

    public void setPublishingFreq(long publishFreq) {
        this.publishFreq = publishFreq;
    }

    protected abstract void setupPublisher(ConnectedNode connectedNode);
    protected abstract void publish();
    protected abstract void setupSubscriber(ConnectedNode connectedNode);

}
