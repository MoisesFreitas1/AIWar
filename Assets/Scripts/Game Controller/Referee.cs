using Accord.MachineLearning.DecisionTrees;
using Accord.MachineLearning.DecisionTrees.Learning;
using UnityEngine;

public class Referee : MonoBehaviour
{
    // Car Objects
    public GameObject CarCreator;
    public GameObject Car;

    // Parameters
    public int n_pods;
    private Color[] carColors;
    private float[][] POD_matrix;
    public float[] PODTimes;
    private double[][] InpThrust, InpSteer;
    private double[][] InpThrustAux, InpSteerAux;
    private int[] OutThrust, OutSteer;
    private int[] OutThrustAux, OutSteerAux;
    private DecisionTree decisionThrust, decisionSteer;
    public int dataSizeTh, dataSizeSt;

    // Time Scale
    public int timeScale; 
    private int timeScaleAnt;

    // Start is called before the first frame update
    void Start()
    {
        timeScale = 1;
        timeScaleAnt = timeScale;
        // Initializing POD times
        PODTimes = new float[n_pods];

        // Generating POD parameters
        CarInstantiate();

        // Initializing Decision Trees
        C45Learning teacher = new C45Learning(new[] {
            DecisionVariable.Continuous("X"),
            DecisionVariable.Continuous("Y"),
            DecisionVariable.Continuous("Z"),
            DecisionVariable.Continuous("W")
        });
        teacher.ParallelOptions.MaxDegreeOfParallelism = 1;

        dataSizeSt = 1;
        dataSizeTh = 1;

        InpThrust = new double[dataSizeTh][];
        InpThrust[0] = new double[4];
        OutThrust = new int[1];
        OutThrust[0] = 1;

        InpSteer = new double[dataSizeSt][];
        InpSteer[0] = new double[4];
        OutSteer = new int[1];
        OutSteer[0] = 0;

        // Use the learning algorithm to induce the tree
        double[][] inputs0 = new double[1][];
        inputs0[0] = new double[4];
        inputs0[0][0] = 0.5f;
        inputs0[0][1] = 10f;
        inputs0[0][2] = 0.5f;
        inputs0[0][3] = 65f;
        int[] outputs0 = new int[1];
        outputs0[0] = 1;
        int[] outputs1 = new int[1];
        outputs1[0] = 0;

        for (int i = 0; i < 4; i++)
        {
            InpThrust[0][i] = i;
            InpSteer[0][i] = i;
        }
        decisionThrust = teacher.Learn(inputs0, outputs0);
        decisionSteer = teacher.Learn(inputs0, outputs1);
    }

    void Update()
    {
        GameObject[] PODs = GameObject.FindGameObjectsWithTag("Player");

        if (PODs.Length == 0)
        {
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
            InpSteer[0] = new double[4];
            OutSteer = new int[1];
            OutSteer[0] = 0;

            for (int i = 0; i < 4; i++)
            {
                InpThrust[0][i] = i;
                InpSteer[0][i] = i;
            }
        }
        else
        {
            for (int i = 0; i < PODs.Length; i++)
            {
                PODTimes[i] = PODs[i].GetComponent<CarControl>().GetCarTime();
            }
        }

        if(timeScale != timeScaleAnt)
        {
            if (PODs.Length != 0)
            {
                for (int i = 0; i < PODs.Length; i++)
                {
                    PODs[i].GetComponent<CarControl>().SetTimeScale(timeScale);
                }
            }
            timeScaleAnt = timeScale;
        }
    }

    // Return Decision Tree
    public DecisionTree GetDecisionThrust()
    {
        return decisionThrust;
    }

    public DecisionTree GetDecisionSteer()
    {
        return decisionSteer;
    }

    // Creates 'n_pods' car instances
    void CarInstantiate()
    {
        ColorGenerator();
        PODGenerator();
        for (int i = 0; i < n_pods; i++)
        {
            PODTimes[i] = 0;
            GameObject car = Instantiate(Car, CarCreator.transform.position, CarCreator.transform.rotation);
            car.GetComponent<CarControl>().SetCarParameters(POD_matrix[i], carColors[i]);
            car.GetComponent<CarControl>().SetTimeScale(timeScale);
        }
    }
    
    // Generate 'n_individuals' different colors
    void ColorGenerator() {
        carColors = new Color[n_pods];
        for (int i = 0; i < n_pods; i++)
        {
            carColors[i] = new Color(Random.value, Random.value, Random.value, 1f);
        }
    }

    // Generate 'n_individuals' pods
    void PODGenerator() {
        POD_matrix = new float[n_pods][];
        for (int i = 0; i < n_pods; i++)
        {
            POD_matrix[i] = new float[7] { 2f, 1f, 10f, 3f, 1f, Random.value, 0f };
        }
    }

    // Set Inputs
    // Thrust
    [SerializeField]
    public void SetInputThrust(double[][] inputThrust, int sizeTh, int[] outputThrust)
    {
        //Debug.Log("sizeTh: " + sizeTh);
        InpThrustAux = new double[dataSizeTh][];
        for(int i = 0; i < dataSizeTh; i++)
        {
            InpThrustAux[i] = new double[4];
            for(int j = 0; j < 4; j++)
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
        for(int i = 0; i < dataSizeTh; i++)
        {
            OutThrustAux[i] = OutThrust[i];
        }
        OutThrust = new int[dataSizeTh + sizeTh];
        for(int i = 0; i < dataSizeTh; i++)
        {
            OutThrust[i] = OutThrustAux[i];
        }
        for(int i = dataSizeTh; i < (dataSizeTh + sizeTh); i++)
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
            InpSteerAux[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                InpSteerAux[i][j] = InpSteer[i][j];
            }
        }

        InpSteer = new double[dataSizeSt + sizeSt][];
        for (int i = 0; i < dataSizeSt; i++)
        {
            InpSteer[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                InpSteer[i][j] = InpSteerAux[i][j];
            }
        }
        for (int i = dataSizeSt; i < (dataSizeSt + sizeSt); i++)
        {
            InpSteer[i] = new double[4];
            for (int j = 0; j < 4; j++)
            {
                //Debug.Log("i - dataSizeSt: " + (i - dataSizeSt));
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
    public DecisionTree DecisionThrust(double[][] inputs, int[] outputs)
    {
        C45Learning teacher = new C45Learning(new[] {
            DecisionVariable.Continuous("X"),
            DecisionVariable.Continuous("Y"),
            DecisionVariable.Continuous("Z"),
            DecisionVariable.Continuous("W")
        });
        teacher.ParallelOptions.MaxDegreeOfParallelism = 1;

        // Use the learning algorithm to induce the tree
        DecisionTree tree = teacher.Learn(inputs, outputs);

        return tree;
    }

    public DecisionTree DecisionSteer(double[][] inputs, int[] outputs)
    {
        C45Learning teacher = new C45Learning(new[] {
            DecisionVariable.Continuous("X"),
            DecisionVariable.Continuous("Y"),
            DecisionVariable.Continuous("Z"),
            DecisionVariable.Continuous("W")
        });
        teacher.ParallelOptions.MaxDegreeOfParallelism = 1;

        // Use the learning algorithm to induce the tree
        DecisionTree tree = teacher.Learn(inputs, outputs);

        return tree;
    }
}
