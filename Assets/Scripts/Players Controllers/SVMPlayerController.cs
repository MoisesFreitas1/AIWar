using Accord.MachineLearning.VectorMachines;
using Accord.Statistics.Kernels;
using UnityEngine;

public class SVMPlayerController : MonoBehaviour
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

    // SVMs
    private SVMController referee;
    private SupportVectorMachine<Gaussian> treeThrust, treeSteer;
    public bool[] answerThrust;
    public bool[] answerSteer;

    //Time scale
    private int timeScale;

    public float EPSILON = 0.1f;
    private float initialPosition;
    public float countTimeStop;

    // -------- FUNCTIONS --------

    // Start is called before the first frame update
    void Start()
    {
        countTimeStop = 0;

        // Initializing sensor parameters
        sensorLength = 10f;
        sensorAngle = 35f;
        frontSensorPosition = 0.1f;
        layer = 1 << LayerMask.NameToLayer("Circuit"); ;

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

        referee = GameObject.FindGameObjectWithTag("SVMController").GetComponent<SVMController>();
        treeThrust = referee.GetDecisionThrust();
        treeSteer = referee.GetDecisionSteer();
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

        if(referee.GetComponent<SVMController>().GetTimesRessurect() == 0)
        {
            if(countTimeStop > 0.5f)
            {
                referee.SetInputThrust(inputThrust, dataSizeTh - 1, outputThrust);
                referee.SetInputSteer(inputSteer, dataSizeSt - 1, outputSteer);
                Destroy(gameObject);
            }
        }

        if ((transform.position.magnitude <= (initialPosition + 0.1f)) && (transform.position.magnitude >= (initialPosition - 0.1f)))
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
            AddDataThrust(sensorLeft, sensorFront, sensorRight, velocity, 0);
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
                AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, forwardSense, 0);
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

        double[][] inputs = new double[1][];
        inputs[0] = new double[5];
        inputs[0][0] = sensorL;
        inputs[0][1] = sensorF;
        inputs[0][2] = sensorR;
        inputs[0][3] = carVelocity;
        inputs[0][4] = forward;

        answerThrust = treeThrust.Decide(inputsT);
        answerSteer = treeSteer.Decide(inputs);

        // Thrust

        if (answerThrust[0] == true)
        {
            thrust = pod[1];
            //Debug.Log("Acelera!");
        }
        else if (answerThrust[0] == false)
        {
            thrust = -pod[2];
            //Debug.Log("Freia!");
        }

        // Steer

        if (answerSteer[0] == true)
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
        else if (answerSteer[0] == false)
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
}
