using Accord.Statistics.Models.Regression;
using Accord.Statistics.Models.Regression.Fitting;
using UnityEngine;

public class LogisticRegressionController : MonoBehaviour
{
    // Car Objects
    public GameObject CarCreator;
    public GameObject Car;
    public int timesRessurect;

    // Parameters
    public int n_pods;
    private float[][] POD_matrix;
    private double[][] InpThrust, InpSteer;
    private double[][] InpThrustAux, InpSteerAux;
    private int[] OutThrust, OutSteer;
    private int[] OutThrustAux, OutSteerAux;
    private LogisticRegression decisionThrust, decisionSteer;
    public int dataSizeTh, dataSizeSt;

    // Time Scale
    public int timeScale;
    private int timeScaleAnt;

    // Start is called before the first frame update
    void Start()
    {
        timeScale = 1;
        timeScaleAnt = timeScale;
        timesRessurect = 0;

        // Generating POD parameters
        CarInstantiate();

        // Initializing Decision Trees
        var teacherT = new IterativeReweightedLeastSquares<LogisticRegression>()
        {
            MaxIterations = 100,
            Regularization = 1e-6
        };

        var teacherS = new IterativeReweightedLeastSquares<LogisticRegression>()
        {
            MaxIterations = 100,
            Regularization = 1e-6
        };

        dataSizeSt = 1;
        dataSizeTh = 1;

        InpThrust = new double[dataSizeTh][];
        InpThrust[0] = new double[4];
        OutThrust = new int[1];
        OutThrust[0] = 1;

        InpSteer = new double[dataSizeSt][];
        InpSteer[0] = new double[5];
        OutSteer = new int[1];
        OutSteer[0] = 0;

        // Use the learning algorithm to induce the tree
        double[][] inputsT0 = new double[1][];
        inputsT0[0] = new double[4];
        inputsT0[0][0] = 0.5f;
        inputsT0[0][1] = 10f;
        inputsT0[0][2] = 0.5f;
        inputsT0[0][3] = 65f;

        double[][] inputsS0 = new double[1][];
        inputsS0[0] = new double[5];
        inputsS0[0][0] = 0.5f;
        inputsS0[0][1] = 10f;
        inputsS0[0][2] = 0.5f;
        inputsS0[0][3] = 65f;
        inputsS0[0][4] = 1f;

        int[] outputs0 = new int[1];
        outputs0[0] = 1;
        int[] outputs1 = new int[1];
        outputs1[0] = 0;

        for (int i = 0; i < 4; i++)
        {
            InpThrust[0][i] = i;
        }
        for (int i = 0; i < 5; i++)
        {
            InpSteer[0][i] = i;
        }
        decisionThrust = teacherT.Learn(inputsT0, outputs0);
        decisionSteer = teacherS.Learn(inputsS0, outputs1);
    }

    public int GetTimesRessurect()
    {
        return timesRessurect;
    }

    void Update()
    {
        GameObject[] PODs = GameObject.FindGameObjectsWithTag("LRPlayer");

        if (PODs.Length == 0)
        {
            timesRessurect = timesRessurect + 1;
            decisionThrust = DecisionThrust(InpThrust, OutThrust);
            decisionSteer = DecisionSteer(InpSteer, OutSteer);
            CarInstantiate();

            dataSizeSt = 1;
            dataSizeTh = 1;

            InpThrust = new double[dataSizeTh][];
            InpThrust[0] = new double[4];
            OutThrust = new int[1];
            OutThrust[0] = 1;

            InpSteer = new double[dataSizeSt][];
            InpSteer[0] = new double[5];
            OutSteer = new int[1];
            OutSteer[0] = 0;

            for (int i = 0; i < 4; i++)
            {
                InpThrust[0][i] = i;
            }
            for (int i = 0; i < 5; i++)
            {
                InpSteer[0][i] = i;
            }
        }

        if (timeScale != timeScaleAnt)
        {
            if (PODs.Length != 0)
            {
                for (int i = 0; i < PODs.Length; i++)
                {
                    PODs[i].GetComponent<LRPlayerController>().SetTimeScale(timeScale);
                }
            }
            timeScaleAnt = timeScale;
        }
    }

    // Return Decision Tree
    public LogisticRegression GetDecisionThrust()
    {
        return decisionThrust;
    }

    public LogisticRegression GetDecisionSteer()
    {
        return decisionSteer;
    }

    // Creates 'n_pods' car instances
    void CarInstantiate()
    {
        PODGenerator();
        for (int i = 0; i < n_pods; i++)
        {
            GameObject car = Instantiate(Car, CarCreator.transform.position, CarCreator.transform.rotation);
            car.GetComponent<LRPlayerController>().SetCarParameters(POD_matrix[i]);
            car.GetComponent<LRPlayerController>().SetTimeScale(timeScale);
        }
    }

    // Generate 'n_individuals' pods
    void PODGenerator()
    {
        POD_matrix = new float[n_pods][];
        for (int i = 0; i < n_pods; i++)
        {
            POD_matrix[i] = new float[7] { 11f, 2f, 10f, 3f, 1f, Random.value, 0f };
        }
    }

    // Set Inputs
    // Thrust
    [SerializeField]
    public void SetInputThrust(double[][] inputThrust, int sizeTh, int[] outputThrust)
    {
        //Debug.Log("sizeTh: " + sizeTh);
        InpThrustAux = new double[dataSizeTh][];
        for (int i = 0; i < dataSizeTh; i++)
        {
            InpThrustAux[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                InpThrustAux[i][j] = InpThrust[i][j];
            }
        }

        InpThrust = new double[dataSizeTh + sizeTh][];
        for (int i = 0; i < dataSizeTh; i++)
        {
            InpThrust[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                InpThrust[i][j] = InpThrustAux[i][j];
            }
        }
        for (int i = dataSizeTh; i < (dataSizeTh + sizeTh); i++)
        {
            InpThrust[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                InpThrust[i][j] = inputThrust[i - dataSizeTh][j];
            }
        }

        OutThrustAux = new int[dataSizeTh];
        for (int i = 0; i < dataSizeTh; i++)
        {
            OutThrustAux[i] = OutThrust[i];
        }
        OutThrust = new int[dataSizeTh + sizeTh];
        for (int i = 0; i < dataSizeTh; i++)
        {
            OutThrust[i] = OutThrustAux[i];
        }
        for (int i = dataSizeTh; i < (dataSizeTh + sizeTh); i++)
        {
            OutThrust[i] = outputThrust[i - dataSizeTh];
        }

        dataSizeTh = dataSizeTh + sizeTh;
    }

    // Steer
    [SerializeField]
    public void SetInputSteer(double[][] inputSteer, int sizeSt, int[] outputSteer)
    {
        //Debug.Log("sizeSt: " + sizeSt);
        InpSteerAux = new double[dataSizeSt][];
        for (int i = 0; i < dataSizeSt; i++)
        {
            InpSteerAux[i] = new double[5];
            for (int j = 0; j < 5; j++)
            {
                InpSteerAux[i][j] = InpSteer[i][j];
            }
        }

        InpSteer = new double[dataSizeSt + sizeSt][];
        for (int i = 0; i < dataSizeSt; i++)
        {
            InpSteer[i] = new double[5];
            for (int j = 0; j < 5; j++)
            {
                InpSteer[i][j] = InpSteerAux[i][j];
            }
        }
        for (int i = dataSizeSt; i < (dataSizeSt + sizeSt); i++)
        {
            InpSteer[i] = new double[5];
            for (int j = 0; j < 5; j++)
            {
                InpSteer[i][j] = inputSteer[i - dataSizeSt][j];
            }
        }

        OutSteerAux = new int[dataSizeSt];
        for (int i = 0; i < dataSizeSt; i++)
        {
            OutSteerAux[i] = OutSteer[i];
        }
        OutSteer = new int[dataSizeSt + sizeSt];
        for (int i = 0; i < dataSizeSt; i++)
        {
            OutSteer[i] = OutSteerAux[i];
        }
        for (int i = dataSizeSt; i < (dataSizeSt + sizeSt); i++)
        {
            OutSteer[i] = outputSteer[i - dataSizeSt];
        }

        dataSizeSt = dataSizeSt + sizeSt;
    }

    // Decision Tree
    public LogisticRegression DecisionThrust(double[][] inputs, int[] outputs)
    {
        var teacher = new IterativeReweightedLeastSquares<LogisticRegression>()
        {
            MaxIterations = 100,
            Regularization = 1e-6
        };

        // Use the learning algorithm to induce the tree
        LogisticRegression tree = teacher.Learn(inputs, outputs);

        return tree;
    }

    public LogisticRegression DecisionSteer(double[][] inputs, int[] outputs)
    {
        var teacher = new IterativeReweightedLeastSquares<LogisticRegression>()
        {
            MaxIterations = 100,
            Regularization = 1e-6
        };

        // Use the learning algorithm to induce the tree
        LogisticRegression tree = teacher.Learn(inputs, outputs);

        return tree;
    }
}
