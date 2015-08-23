package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import rosjava_test_msgs.AddTwoIntsRequest;
import rosjava_test_msgs.AddTwoIntsResponse;
import std_msgs.Bool;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class CalibrationPositionNode implements NodeMain {

    private String nodeName = "customButton";
    private Publisher<Bool> publisher = null;
    private int counter = 0;

    private boolean isServer;
    private boolean isClient;

    private boolean client_bool;
    private boolean server_bool;

    private String serverTopic;
    private String clientTopic;

    private boolean hasServerMsg =false;
    private boolean hasClientMsg =false;

    public void serverOf(String topic){
        isServer =true;
        serverTopic =topic;
    }

    public void clientOf(String topic){
        isClient =true;
        clientTopic =topic;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(isClient){
            publisher = connectedNode.newPublisher(clientTopic, Bool._TYPE);
            publish(connectedNode);
        }
        if(isServer){
            server(connectedNode);
        }
    }


    private void publish(final ConnectedNode connectedNode){
        ServiceClient<AddTwoIntsRequest, AddTwoIntsResponse> serviceClient;

        try {
            serviceClient = connectedNode.newServiceClient("add_two_ints", rosjava_test_msgs.AddTwoInts._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }
        final rosjava_test_msgs.AddTwoIntsRequest request = serviceClient.newMessage();
        request.setA(2);
        request.setB(2);
        serviceClient.call(request, new ServiceResponseListener<AddTwoIntsResponse>() {
            @Override
            public void onSuccess(rosjava_test_msgs.AddTwoIntsResponse response) {
                connectedNode.getLog().info(
                        String.format("%d + %d = %d", request.getA(), request.getB(), response.getSum()));
            }
            @Override
            public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
            }
        });
    }

    private void server(ConnectedNode connectedNode) {

        connectedNode.newServiceServer("add_two_ints",
                rosjava_test_msgs.AddTwoInts._TYPE,
                new ServiceResponseBuilder<AddTwoIntsRequest, AddTwoIntsResponse>() {
                    @Override
                    public void
                    build(rosjava_test_msgs.AddTwoIntsRequest request, rosjava_test_msgs.AddTwoIntsResponse response) {

                        response.setSum(request.getA() + request.getB());

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

    public boolean isClient_bool() {
        return client_bool;
    }

    public void setClient_bool(boolean client_bool) {
        this.client_bool = client_bool;
    }

    public boolean isServer_bool() {
        return server_bool;
    }

    public void setServer_bool(boolean server_bool) {
        this.server_bool = server_bool;
    }

    public boolean hasReceivedMsg() {
        return hasServerMsg;
    }

    public void setHasServerMsg(boolean hasServerMsg) {
        this.hasServerMsg = hasServerMsg;
    }

    public boolean hasPublishedMsg() {
        return hasClientMsg;
    }

    public void setHasClientMsg(boolean hasClientMsg) {
        this.hasClientMsg = hasClientMsg;
    }

}
