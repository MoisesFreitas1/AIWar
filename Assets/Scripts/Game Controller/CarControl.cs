using Accord.MachineLearning.DecisionTrees;
using UnityEngine;

public class CarControl : MonoBehaviour
{

    // Tune only the public parameters
    // Vref - Up - Down - Steer - Smax - Smin - L-R
    public float[] pod;
    public Color carColor;

    // User defined sensor parameters
    private float sensorLength;
    private float sensorAngle;
    private float frontSensorPosition;
    private int layer;


    // Lap time
    private float carTime;
    //public int finish;

    // Measurements
    public float sensorLeft;
    public float sensorRight;
    public float sensorFront;
    public float velocity;

    // Thrust and steer commands
    private float thrust;
    private float steer;

    // Rigid Body and Sprite Renderer component
    private Rigidbody2D rb;
    private SpriteRenderer rend;

    // Add data
    public int dataSizeTh, dataSizeSt;
    private int n_col;
    private double[][] inputThrust, inputThrustAux;
    private double[][] inputSteer, inputSteerAux;
    private int[] outputThrust, outputThrustAux;
    private int[] outputSteer, outputSteerAux;

    // Decision Trees
    private Referee referee;
    private DecisionTree treeThrust, treeSteer;

    //Time scale
    private int timeScale;

    public float EPSILON = 0.1f;

    // -------- FUNCTIONS --------

    // Start is called before the first frame update
    void Start()
    {
        // Initializing sensor parameters
        sensorLength = 10f;
        sensorAngle = 35f;
        frontSensorPosition = 0.23f;
        layer = 1 << LayerMask.NameToLayer("Circuit"); ;

        // Initializing car parameters
        rb = GetComponent<Rigidbody2D>();
        rb.mass = 3f;
        rb.drag = 1f;
        rb.angularDrag = 1f;

        // Initializing Sprite parameters
        rend = GetComponent<SpriteRenderer>();
        rend.color = carColor;

        // Have you finish the lap?
        //finish = 0;

        n_col = 4;
        dataSizeTh = 1;
        dataSizeSt = 1;

        referee = GameObject.FindGameObjectWithTag("GameController").GetComponent<Referee>();
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
        //float randomSample = Random.Range(0f, 100f);
        //if(randomSample < 100f)
        //{
        // Where is the car pointing?
        Vector3 forward = transform.TransformDirection(Vector3.up) * sensorLength;

        // Where the sensor must be in the car?
        Vector3 sensorStartPos = transform.position;
        sensorStartPos += transform.up * frontSensorPosition;

        // Getting the sensor distance measurements
        sensorLeft = SensorRay(sensorStartPos, Quaternion.AngleAxis(sensorAngle, transform.forward) * forward, sensorLength);
        sensorFront = SensorRay(sensorStartPos, forward, sensorLength);
        sensorRight = SensorRay(sensorStartPos, Quaternion.AngleAxis(-sensorAngle, transform.forward) * forward, sensorLength);

        //if (sensorLeft == 0f)
        //{
        //    sensorLeft = sensorLength;
        //}
        //if (sensorFront == 0f)
        //{
        //    sensorFront = sensorLength;
        //}
        //if (sensorRight == 0f)
        //{
        //    sensorRight = sensorLength;
        //}

        // Add data in input vectors
        velocity = rb.velocity.magnitude;
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
            AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, 1);
        }
        else if (Mathf.Abs(sensorLeft - sensorRight) < EPSILON)
        {
            AddDataSteer(sensorLeft, sensorFront, sensorRight, velocity, 0);
        }

        dataSizeSt = dataSizeSt + 1;

        // Computing control commands based on sensor measurements and car velocity
        ComputeControl(sensorLeft, sensorFront, sensorRight, velocity);
        //}


        // Time alive
        carTime += Time.fixedDeltaTime;
    }

    // In case the car has crashed or crossed the Finish lap.
    void OnTriggerEnter2D(Collider2D other)
    {
        // Has it crashed?
        if (other.gameObject.tag.Equals("Circuit"))
        {
            referee.SetInputThrust(inputThrust, dataSizeTh - 2, outputThrust);
            referee.SetInputSteer(inputSteer, dataSizeSt - 2, outputSteer);
            Destroy(gameObject);
        }
        //if (other.gameObject.tag.Equals("LAP"))
        //{
        //    //referee.SetInputThrust(inputThrust, dataSizeTh - 2, outputThrust);
        //    //referee.SetInputSteer(inputSteer, dataSizeSt - 2, outputSteer);
        //    Destroy(gameObject);
        //}
    }

    public void SetCarParameters(float[] UserPod, Color UserColor)
    {
        pod = UserPod;
        carColor = UserColor;
    }

    // Sensor measurements
    float SensorRay(Vector2 origin, Vector2 direction, float distance)
    {
        RaycastHit2D hit = Physics2D.Raycast(origin, direction, distance, layer);

        if (hit.collider != null)
        {
            // Draw a line iff you have detected something in the sensor range
            // CAN BE REMOVED
            Debug.DrawRay(origin, direction, Color.blue);
        }
        return hit.distance;
    }

    // Compute car control based on sensor readings
    void ComputeControl(float sensorL, float sensorF, float sensorR, float carVelocity)
    {
        // Inputs
        float[][] inputs = new float[1][];
        inputs[0] = new float[4];
        inputs[0][0] = sensorL;
        inputs[0][1] = sensorF;
        inputs[0][2] = sensorR;
        inputs[0][3] = carVelocity;

        // Thrust
        int[] answerThrust = treeThrust.Decide(inputs);
        int[] answerSteer = treeSteer.Decide(inputs);

        if (answerThrust[0] == 1)
        {
            thrust = pod[1];
            //Debug.Log("Acelera!");
        }
        else if (answerThrust[0] == 0)
        {
            thrust = -pod[2];
            //Debug.Log("Freia!");
        }

        //// Steer

        if (answerSteer[0] == 1)
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
        else if (answerSteer[0] == 0)
        {
            steer = 0;
            //Debug.Log("Vai reto!");
        }

        // Command
        rb.AddRelativeForce(new Vector2(0f, thrust));
        rb.AddTorque(steer);

    }

    // Car lap time
    public float GetCarTime()
    {
        return carTime;
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
            for (int j = 0; j < n_col; j++)
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
    public void AddDataSteer(float sensorL, float sensorF, float sensorR, float carVelocity, int output)
    {
        inputSteerAux = inputSteer;
        outputSteerAux = outputSteer;
        inputSteer = new double[dataSizeSt][];
        outputSteer = new int[dataSizeSt];
        for (int i = 0; i < (dataSizeSt - 1); i++)
        {
            outputSteer[i] = outputSteerAux[i];
            inputSteer[i] = new double[4];
            for (int j = 0; j < n_col; j++)
            {
                inputSteer[i][j] = inputSteerAux[i][j];
            }
        }
        inputSteer[dataSizeSt - 1] = new double[4];
        inputSteer[dataSizeSt - 1][0] = sensorL;
        inputSteer[dataSizeSt - 1][1] = sensorF;
        inputSteer[dataSizeSt - 1][2] = sensorR;
        inputSteer[dataSizeSt - 1][3] = carVelocity;
        outputSteer[dataSizeSt - 1] = output;
    }

    public void SetTimeScale(int scale)
    {
        timeScale = scale;
    }
}
