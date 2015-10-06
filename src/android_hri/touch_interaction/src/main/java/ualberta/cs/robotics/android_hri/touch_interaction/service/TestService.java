package ualberta.cs.robotics.android_hri.touch_interaction.service;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.service.ServiceServer;

import rosjava_test_msgs.AddTwoInts;
import rosjava_test_msgs.AddTwoIntsRequest;
import rosjava_test_msgs.AddTwoIntsResponse;

/**
 * Created by dhrodriguezg on 7/29/15.
 */

public class TestService extends AbstractService {

    private long a;
    private long b;
    private long sum;
    protected AddTwoIntsRequest request = null;
    protected ServiceClient<AddTwoIntsRequest, AddTwoIntsResponse> serviceClient = null;
    protected ServiceServer<AddTwoIntsRequest, AddTwoIntsResponse> serviceServer = null;

    protected void setupClient(final ConnectedNode connectedNode) {
        try {
            serviceClient = connectedNode.newServiceClient(clientTopic, AddTwoInts._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }
    }

    protected void publish() {
        request = serviceClient.newMessage();
        request.setA(a);
        request.setB(b);
        serviceClient.call(request, new ServiceResponseListener<AddTwoIntsResponse>() {
            @Override
            public void onSuccess(AddTwoIntsResponse response) {
                sum=response.getSum();
            }
            @Override
            public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
            }
        });
    }

    protected void setupServer(ConnectedNode connectedNode) {
        serviceServer = connectedNode.newServiceServer(serverTopic, AddTwoInts._TYPE, new ServiceResponseBuilder<AddTwoIntsRequest, AddTwoIntsResponse>() {
            @Override
            public void build(AddTwoIntsRequest request, AddTwoIntsResponse response) {
                response.setSum(request.getA() + request.getB());
            }
        });
    }

    public long getA() {
        return a;
    }

    public void setA(long a) {
        this.a = a;
    }

    public long getB() {
        return b;
    }

    public void setB(long b) {
        this.b = b;
    }

    public long getSum() {
        return sum;
    }

    public void setSum(long sum) {
        this.sum = sum;
    }
}
