using Accord.Math;
using Accord.Neuro;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class NNPlayerController : MonoBehaviour
{
    // Tune only the public parameters
    // Vref - Up - Down - Steer - Smax - Smin - L-R
    public float[] pod;

    // User defined sensor parameters
    private float sensorLength;
    private float sensorAngle;
    private float frontSensorPosition;
    private int layer;

    // Measurements
    public float sensorLeft;
    public float sensorRight;
    public float sensorFront;
    public float velocity;
    public float currentCheckpoint;
    public float previewCheckpoint;
    public float forwardSense;

    // Thrust and steer commands
    private float thrust;
    private float steer;

    // Rigid Body and Sprite Renderer component
    private Rigidbody2D rb;
    // Add data
    public int dataSizeTh, dataSizeSt;
    private double[][] inputThrust, inputThrustAux;
    private double[][] inputSteer, inputSteerAux;
    private int[] outputThrust, outputThrustAux;
    private int[] outputSteer, outputSteerAux;

    // Decision Trees
    private NeuralNetworkController referee;
    private ActivationNetwork treeThrust, treeSteer;
    public double answerThrust1, answerThrust2;
    public double answerSteer1, answerSteer2;
    public double[] weightsThrust, weightsSteer;
    public double[] thetaThrust, thetaSteer;

    //Time scale
    private int timeScale;
    public float lap;
    public float currentlap;


    public float EPSILON = 0.1f;
    private float initialPosition;
    private float countTimeStop;

    // -------- FUNCTIONS --------

    // Start is called before the first frame update
    void Start()
    {
        countTimeStop = 0;

        // Initializing sensor parameters
        sensorLength = 10f;
        sensorAngle = 35f;
        frontSensorPosition = 0.1f;
        layer = 1 << LayerMask.NameToLayer("Circuit");
        lap = 1;

        // Initializing car parameters
        rb = GetComponent<Rigidbody2D>();
        rb.mass = 3f;
        rb.drag = 1f;
        rb.angularDrag = 1f;

        //Checkpoints
        currentCheckpoint = 1;
        previewCheckpoint = 0;

        // Have you finish the lap?

        dataSizeTh = 1;
        dataSizeSt = 1;

        referee = GameObject.FindGameObjectWithTag("NNController").GetComponent<NeuralNetworkController>();
        weightsThrust = referee.GetDecisionThrust();
        weightsSteer = referee.GetDecisionSteer();
        thetaThrust = referee.GetDecisionThrustTheta();
        thetaSteer = referee.GetDecisionSteerTheta();
    }

    private void Update()
    {
        Time.timeScale = timeScale;
    }

    // Keep going every frame...
    void FixedUpdate()
    {

        if (Mathf.Abs(countTimeStop) < EPSILON)
        {
            initialPosition = transform.position.magnitude;
        }
        countTimeStop = countTimeStop + Time.fixedDeltaTime;

        if (referee.GetComponent<NeuralNetworkController>().GetTimesRessurect() == 0)
        {
            if (countTimeStop > 0.5f)
            {
                referee.SetInputThrust(inputThrust, dataSizeTh - 1, outputThrust);
                referee.SetInputSteer(inputSteer, dataSizeSt - 1, outputSteer);
                Destroy(gameObject);
            }
        }

        if ((transform.position.magnitude <= (initialPosition + 0.5f)) && (transform.position.magnitude >= (initialPosition - 0.5f)))
        {
            if (countTimeStop > 10)
            {
                referee.SetInputThrust(inputThrust, dataSizeTh - 1, outputThrust);
                referee.SetInputSteer(inputSteer, dataSizeSt - 1, outputSteer);
                Destroy(gameObject);
            }
        }
        else
        {
            countTimeStop = 0f;
        }

        // Where is the car pointing?
        Vector3 forward = transform.TransformDirection(Vector3.up) * sensorLength;

        // Where the sensor must be in the car?
        Vector3 sensorStartPos = transform.position;
        sensorStartPos += transform.up * frontSensorPosition;

        // Getting the sensor distance measurements
        sensorLeft = SensorRay(sensorStartPos, Quaternion.AngleAxis(sensorAngle, transform.forward) * forward, sensorLength);
        sensorFront = SensorRay(sensorStartPos, forward, sensorLength);
        sensorRight = SensorRay(sensorStartPos, Quaternion.AngleAxis(-sensorAngle, transform.forward) * forward, sensorLength);

        // Add data in input vectors
        velocity = rb.velocity.magnitude;
        forwardSense = currentCheckpoint - previewCheckpoint;

        // Thrust
        if ((sensorFront > pod[4] && velocity < pod[0]) || Mathf.Abs(sensorLeft - sensorRight) < EPSILON)
        {
            AddDataThrust(sensorLeft, sensorFront, sensorRight, velocity, 1);
        }
        else if (sensorFront < pod[4] || velocity > pod[0])
        {
            AddDataThrust(sensorLeft, sensorFront, sensorRight, velocity, -1);
        }
        dataSizeTh = dataSizeTh + 1;

        // Steer
        if ((sensorLeft > sensorRight) || (sensorLeft < sensorRight))
        {
            AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, forwardSense, 1);
        }
        else if (Mathf.Abs(sensorLeft - sensorRight) < EPSILON)
        {
            if (currentCheckpoint > previewCheckpoint)
            {
                AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, forwardSense, -1);
            }
            else
            {
                AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, forwardSense, 1);
            }
        }
        dataSizeSt = dataSizeSt + 1;

        // Computing control commands based on sensor measurements and car velocity
        ComputeControl(sensorLeft, sensorFront, sensorRight, velocity, forwardSense);

        // Time alive
    }

    // In case the car has crashed or crossed the Finish lap.
    void OnTriggerEnter2D(Collider2D other)
    {
        // Has it crashed?
        if (other.gameObject.tag.Equals("Circuit"))
        {
            referee.SetInputThrust(inputThrust, dataSizeTh - 1, outputThrust);
            referee.SetInputSteer(inputSteer, dataSizeSt - 1, outputSteer);
            Destroy(gameObject);
        }
        if (other.gameObject.tag.Equals("Checkpoint"))
        {
            if (System.Math.Abs(other.gameObject.GetComponent<CheckPointsController>().GetCpID() - currentCheckpoint) > EPSILON)
            {
                previewCheckpoint = currentCheckpoint;
                currentCheckpoint = other.gameObject.GetComponent<CheckPointsController>().GetCpID();
            }
        }
        if (other.gameObject.tag.Equals("Lap"))
        {
            currentlap = other.gameObject.GetComponent<CheckPointsController>().GetCpID();
            if (currentlap > lap)
            {
                float timelap = referee.GetTimeLap();
                referee.SetNLaps();
                int nlap = referee.GetNLaps();
                Debug.Log("Neural Network \nLap: " + nlap + "   Time: " + timelap);
            }
            lap = currentlap;
        }
    }

    public void SetCarParameters(float[] UserPod)
    {
        pod = UserPod;
    }

    // Sensor measurements
    float SensorRay(Vector2 origin, Vector2 direction, float distance)
    {
        RaycastHit2D hit = Physics2D.Raycast(origin, direction, distance, layer);
        if (hit.collider != null)
        {
            // Draw a line if you have detected something in the sensor range
            Debug.DrawRay(origin, direction, Color.blue);
        }
        return hit.distance;
    }

    // Compute car control based on sensor readings
    void ComputeControl(float sensorL, float sensorF, float sensorR, float carVelocity, float forward)
    {
        // Inputs
        double[][] inputsT = new double[1][];
        inputsT[0] = new double[4];
        inputsT[0][0] = sensorL;
        inputsT[0][1] = sensorF;
        inputsT[0][2] = sensorR;
        inputsT[0][3] = carVelocity;

        DecisionThrust(inputsT);

        double[][] inputsS = new double[1][];
        inputsS[0] = new double[5];
        inputsS[0][0] = sensorL;
        inputsS[0][1] = sensorF;
        inputsS[0][2] = sensorR;
        inputsS[0][3] = carVelocity;
        inputsS[0][4] = forward;

        DecisionSteer(inputsS);

        // Thrust

        if (answerThrust1 > answerThrust2)
        {
            thrust = pod[1];
            //Debug.Log("Acelera!");
        }
        else if (answerThrust1 < answerThrust2)
        {
            thrust = -pod[2];
            //Debug.Log("Freia!");
        }

        // Steer

        if (answerSteer1 > answerSteer2)
        {
            if (sensorLeft > sensorRight)
            {
                steer = pod[3];
                //Debug.Log("Vire a Esquerda!");
            }
            else
            {
                steer = -pod[3];
                //Debug.Log("Vire a Direita!");
            }
        }
        else if (answerSteer1 < answerSteer2)
        {
            steer = 0;
            //Debug.Log("Vai reto!");
        }

        // Command
        rb.AddRelativeForce(new Vector2(0f, thrust));
        rb.AddTorque(steer);

    }

    [SerializeField]
    public void AddDataThrust(float sensorL, float sensorF, float sensorR, float carVelocity, int output)
    {
        inputThrustAux = inputThrust;
        outputThrustAux = outputThrust;
        inputThrust = new double[dataSizeTh][];
        outputThrust = new int[dataSizeTh];
        for (int i = 0; i < (dataSizeTh - 1); i++)
        {
            outputThrust[i] = outputThrustAux[i];
            inputThrust[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                inputThrust[i][j] = inputThrustAux[i][j];
            }
        }
        inputThrust[dataSizeTh - 1] = new double[4];
        inputThrust[dataSizeTh - 1][0] = sensorL;
        inputThrust[dataSizeTh - 1][1] = sensorF;
        inputThrust[dataSizeTh - 1][2] = sensorR;
        inputThrust[dataSizeTh - 1][3] = carVelocity;
        outputThrust[dataSizeTh - 1] = output;
    }

    [SerializeField]
    public void AddDataSteer(float sensorL, float sensorF, float sensorR, float carVelocity, float forward, int output)
    {
        inputSteerAux = inputSteer;
        outputSteerAux = outputSteer;
        inputSteer = new double[dataSizeSt][];
        outputSteer = new int[dataSizeSt];
        for (int i = 0; i < (dataSizeSt - 1); i++)
        {
            outputSteer[i] = outputSteerAux[i];
            inputSteer[i] = new double[5];
            for (int j = 0; j < 5; j++)
            {
                inputSteer[i][j] = inputSteerAux[i][j];
            }
        }
        inputSteer[dataSizeSt - 1] = new double[5];
        inputSteer[dataSizeSt - 1][0] = sensorL;
        inputSteer[dataSizeSt - 1][1] = sensorF;
        inputSteer[dataSizeSt - 1][2] = sensorR;
        inputSteer[dataSizeSt - 1][3] = carVelocity;
        inputSteer[dataSizeSt - 1][4] = forward;
        outputSteer[dataSizeSt - 1] = output;
    }

    public void SetTimeScale(int scale)
    {
        timeScale = scale;
    }

    public void DecisionThrust(double[][] inputs)
    {
        double net1 = (weightsThrust[0] * inputs[0][0]) + (weightsThrust[1] * inputs[0][1]) + (weightsThrust[2] * inputs[0][2]) + (weightsThrust[3] * inputs[0][3]) + thetaSteer[0];
        double net2 = (weightsThrust[4] * inputs[0][0]) + (weightsThrust[5] * inputs[0][1]) + (weightsThrust[6] * inputs[0][2]) + (weightsThrust[7] * inputs[0][3]) + thetaSteer[1];

        double outIntern11 = FunctionTransfer(net1);
        double outIntern12 = FunctionTransfer(net2);

        double net3 = (weightsThrust[8] * outIntern11) + (weightsThrust[9] * outIntern12) + thetaSteer[2];
        double net4 = (weightsThrust[10] * outIntern11) + (weightsThrust[11] * outIntern12) + thetaSteer[3];
        double net5 = (weightsThrust[12] * outIntern11) + (weightsThrust[13] * outIntern12) + thetaSteer[4];

        double outIntern21 = FunctionTransfer(net3);
        double outIntern22 = FunctionTransfer(net4);
        double outIntern23 = FunctionTransfer(net5);

        double net6 = (weightsThrust[14] * outIntern21) + (weightsThrust[15] * outIntern22) + (weightsThrust[16] * outIntern23) + thetaSteer[5];
        double net7 = (weightsThrust[17] * outIntern21) + (weightsThrust[18] * outIntern22) + (weightsThrust[19] * outIntern23) + thetaSteer[6];

        answerThrust1 = FunctionTransfer(net6);
        answerThrust2 = FunctionTransfer(net7);
    }

    public void DecisionSteer(double[][] inputs)
    {
        double net1 = (weightsSteer[0] * inputs[0][0]) + (weightsSteer[1] * inputs[0][1]) + (weightsSteer[2] * inputs[0][2]) + (weightsSteer[3] * inputs[0][3]) + thetaSteer[0];
        double net2 = (weightsSteer[4] * inputs[0][0]) + (weightsSteer[5] * inputs[0][1]) + (weightsSteer[6] * inputs[0][2]) + (weightsSteer[7] * inputs[0][3]) + thetaSteer[1];

        double outIntern11 = FunctionTransfer(net1);
        double outIntern12 = FunctionTransfer(net2);

        double net3 = (weightsSteer[8] * outIntern11) + (weightsSteer[9] * outIntern12) + thetaSteer[2];
        double net4 = (weightsSteer[10] * outIntern11) + (weightsSteer[11] * outIntern12) + thetaSteer[3];
        double net5 = (weightsSteer[12] * outIntern11) + (weightsSteer[13] * outIntern12) + thetaSteer[4];

        double outIntern21 = FunctionTransfer(net3);
        double outIntern22 = FunctionTransfer(net4);
        double outIntern23 = FunctionTransfer(net5);

        double net6 = (weightsSteer[14] * outIntern21) + (weightsSteer[15] * outIntern22) + (weightsSteer[16] * outIntern23) + thetaSteer[5];
        double net7 = (weightsSteer[17] * outIntern21) + (weightsSteer[18] * outIntern22) + (weightsSteer[19] * outIntern23) + thetaSteer[6];

        answerSteer1 = FunctionTransfer(net6);
        answerSteer2 = FunctionTransfer(net7);
    }

    private double FunctionTransfer(double x)
    {
        double f = (1) / (1 + Mathf.Exp((float)-x));
        return f;
    }


}
