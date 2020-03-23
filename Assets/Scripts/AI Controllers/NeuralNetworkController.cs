using UnityEngine;
using Random = UnityEngine.Random;

public class NeuralNetworkController : MonoBehaviour
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
    public int dataSizeTh, dataSizeSt;
    private double[] weightsThrust, weightsSteer;
    private int n_weightsThrust, n_weightsSteer;
    private double[] thetaThrust, thetaSteer;
    private float alpha;
    private float EPSILON = 0.1f;


    // Time Scale
    public int timeScale;
    private int timeScaleAnt;
    private int nLaps;
    private float timeLap;

    // Start is called before the first frame update
    void Start()
    {
        timeScale = 1;
        timeScaleAnt = timeScale;
        timesRessurect = 0;
        nLaps = -1;
        timeLap = 0;

        // Initializing POD times
        // Generating POD parameters
        CarInstantiate();

        alpha = 0.001f; // muito bom

        dataSizeSt = 1;
        dataSizeTh = 1;

        InpThrust = new double[dataSizeTh][];
        InpThrust[0] = new double[4];
        OutThrust = new int[1];
        OutThrust[0] = 1;

        InpSteer = new double[dataSizeSt][];
        InpSteer[0] = new double[5];
        OutSteer = new int[1];
        OutSteer[0] = -1;

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
        outputs1[0] = -1;

        for (int i = 0; i < 4; i++)
        {
            InpThrust[0][i] = i;
        }
        for (int i = 0; i < 5; i++)
        {
            InpSteer[0][i] = i;
        }

        // Initializing Neural Network's weights
        // Thrust
        n_weightsThrust = 20;
        weightsThrust = new double[n_weightsThrust];
        for (int j = 0; j < n_weightsThrust; j++)
        {
            weightsThrust[j] = Random.Range(-0.8f, 0.8f);
        }

        thetaThrust = new double[7];
        for (int i = 0; i < thetaThrust.Length; i++)
        {
            thetaThrust[i] = Random.Range(-0.8f, 0.8f);
        }

        // Steer
        n_weightsSteer = 22;
        weightsSteer = new double[n_weightsSteer];
        for (int j = 0; j < n_weightsSteer; j++)
        {
            weightsSteer[j] = Random.Range(-0.8f, 0.8f);
        }

        thetaSteer = new double[7];
        for (int i = 0; i < thetaSteer.Length; i++)
        {
            thetaSteer[i] = Random.Range(-0.8f, 0.8f);
        }
    }

    public int GetTimesRessurect()
    {
        return timesRessurect;
    }

    void Update()
    {
        timeLap = timeLap + Time.deltaTime;
        GameObject[] PODs = GameObject.FindGameObjectsWithTag("NNPlayer");

        if (PODs.Length == 0)
        {
            timesRessurect = timesRessurect + 1;
            weightsThrust = DecisionThrust(InpThrust, OutThrust);
            weightsSteer = DecisionSteer(InpSteer, OutSteer);
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
            OutSteer[0] = -1;

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
                    PODs[i].GetComponent<NNPlayerController>().SetTimeScale(timeScale);
                }
            }
            timeScaleAnt = timeScale;
        }
    }

    // Return Decision Tree
    public double[] GetDecisionThrust()
    {
        return weightsThrust;
    }

    public double[] GetDecisionSteer()
    {
        return weightsSteer;
    }

    public double[] GetDecisionThrustTheta()
    {
        return thetaThrust;
    }

    public double[] GetDecisionSteerTheta()
    {
        return thetaSteer;
    }

    // Counter Laps and Time
    public float GetTimeLap()
    {
        return timeLap;
    }

    public void SetNLaps()
    {
        nLaps = nLaps + 1;
    }

    public int GetNLaps()
    {
        return nLaps;
    }

    // Creates 'n_pods' car instances
    void CarInstantiate()
    {
        PODGenerator();
        for (int i = 0; i < n_pods; i++)
        {
            GameObject car = Instantiate(Car, CarCreator.transform.position, CarCreator.transform.rotation);
            car.GetComponent<NNPlayerController>().SetCarParameters(POD_matrix[i]);
            car.GetComponent<NNPlayerController>().SetTimeScale(timeScale);
        }
    }

    // Generate 'n_individuals' pods
    void PODGenerator()
    {
        POD_matrix = new float[n_pods][];
        for (int i = 0; i < n_pods; i++)
        {
            POD_matrix[i] = new float[7] { 10f, 2f, 10f, 3f, 1f, Random.value, 0f };
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

    // Neural Network
    public double[] DecisionThrust(double[][] inputs, int[] outputs)
    {
        for (int i = 0; i < inputs.Length; i++)
        {
            double net1 = (weightsThrust[0] * inputs[i][0]) + (weightsThrust[1] * inputs[i][1]) + (weightsThrust[2] * inputs[i][2]) + (weightsThrust[3] * inputs[i][3]) + thetaThrust[0];
            double net2 = (weightsThrust[4] * inputs[i][0]) + (weightsThrust[5] * inputs[i][1]) + (weightsThrust[6] * inputs[i][2]) + (weightsThrust[7] * inputs[i][3]) + thetaThrust[1];

            double outIntern11 = FunctionTransfer(net1);
            double outIntern12 = FunctionTransfer(net2);

            double net3 = (weightsThrust[8] * outIntern11) + (weightsThrust[9] * outIntern12) + thetaThrust[2];
            double net4 = (weightsThrust[10] * outIntern11) + (weightsThrust[11] * outIntern12) + thetaThrust[3];
            double net5 = (weightsThrust[12] * outIntern11) + (weightsThrust[13] * outIntern12) + thetaThrust[4];

            double outIntern21 = FunctionTransfer(net3);
            double outIntern22 = FunctionTransfer(net4);
            double outIntern23 = FunctionTransfer(net5);

            double net6 = (weightsThrust[14] * outIntern21) + (weightsThrust[15] * outIntern22) + (weightsThrust[16] * outIntern23) + thetaThrust[5];
            double net7 = (weightsThrust[17] * outIntern21) + (weightsThrust[18] * outIntern22) + (weightsThrust[19] * outIntern23) + thetaThrust[6];

            double outExtern1 = FunctionTransfer(net6);
            double outExtern2 = FunctionTransfer(net7);

            double[] deltaks = new double[2];

            if ((inputs[i][1] > 1f && inputs[i][3] < 5f) || Mathf.Abs((float)(inputs[i][0] - inputs[i][2])) < EPSILON)
            {
                // acelerar
                deltaks[0] = (0.6f - outExtern1) * DotFunctionTransfer(net3); // acelerar
                deltaks[1] = (0.4f - outExtern2) * DotFunctionTransfer(net4); // desacelerar
            }
            else if (inputs[i][1] < 1f || inputs[i][3] > 5f)
            {
                // desacelerar
                deltaks[0] = (0.4f - outExtern1) * DotFunctionTransfer(net3); // acelerar
                deltaks[1] = (0.6f - outExtern2) * DotFunctionTransfer(net4); // desacelerar
            }

            thetaThrust[5] = thetaThrust[5] + alpha * deltaks[0];
            thetaThrust[6] = thetaThrust[6] + alpha * deltaks[1];

            double[] Er = new double[2];
            Er[0] = (Mathf.Pow((float)deltaks[0], 2)) / 2f;
            Er[1] = (Mathf.Pow((float)deltaks[1], 2)) / 2f;
            double er = Er[0] + Er[1];

            if (er > EPSILON)
            {
                double[] delta1 = new double[3];
                delta1[0] = DotFunctionTransfer(net3) * (deltaks[0] * weightsThrust[14] + deltaks[1] * weightsThrust[17]);
                delta1[1] = DotFunctionTransfer(net4) * (deltaks[0] * weightsThrust[15] + deltaks[1] * weightsThrust[18]);
                delta1[2] = DotFunctionTransfer(net5) * (deltaks[0] * weightsThrust[16] + deltaks[1] * weightsThrust[19]);

                weightsThrust[8] = weightsThrust[8] + alpha * delta1[0] * outIntern11;
                weightsThrust[9] = weightsThrust[9] + alpha * delta1[0] * outIntern12;

                weightsThrust[10] = weightsThrust[10] + alpha * delta1[1] * outIntern11;
                weightsThrust[11] = weightsThrust[11] + alpha * delta1[1] * outIntern12;

                weightsThrust[12] = weightsThrust[12] + alpha * delta1[2] * outIntern11;
                weightsThrust[13] = weightsThrust[13] + alpha * delta1[2] * outIntern12;

                thetaThrust[2] = thetaThrust[2] + alpha * delta1[0];
                thetaThrust[3] = thetaThrust[3] + alpha * delta1[1];
                thetaThrust[4] = thetaThrust[4] + alpha * delta1[2];


                double[] delta0 = new double[2];
                delta0[0] = DotFunctionTransfer(net1) * (delta1[0] * weightsThrust[8] + delta1[1] * weightsThrust[10] + delta1[2] * weightsThrust[12]);
                delta0[1] = DotFunctionTransfer(net2) * (delta1[0] * weightsThrust[9] + delta1[1] * weightsThrust[11] + delta1[2] * weightsThrust[13]);

                weightsThrust[0] = weightsThrust[0] + alpha * delta0[0] * inputs[i][0];
                weightsThrust[1] = weightsThrust[1] + alpha * delta0[0] * inputs[i][1];
                weightsThrust[2] = weightsThrust[2] + alpha * delta0[0] * inputs[i][2];
                weightsThrust[3] = weightsThrust[3] + alpha * delta0[0] * inputs[i][3];

                weightsThrust[4] = weightsThrust[4] + alpha * delta0[1] * inputs[i][0];
                weightsThrust[5] = weightsThrust[5] + alpha * delta0[1] * inputs[i][1];
                weightsThrust[6] = weightsThrust[6] + alpha * delta0[1] * inputs[i][2];
                weightsThrust[7] = weightsThrust[7] + alpha * delta0[1] * inputs[i][3];

                thetaThrust[0] = thetaThrust[0] + alpha * delta0[0];
                thetaThrust[1] = thetaThrust[1] + alpha * delta0[1];
            }
        }
        return weightsThrust;
    }

    public double[] DecisionSteer(double[][] inputs, int[] outputs)
    {
        for (int i = 0; i < inputs.Length; i++)
        {
            double net1 = (weightsSteer[0] * inputs[i][0]) + (weightsSteer[1] * inputs[i][1]) + (weightsSteer[2] * inputs[i][2]) + (weightsSteer[3] * inputs[i][3]) + (weightsSteer[4] * inputs[i][4]) + thetaSteer[0];
            double net2 = (weightsSteer[5] * inputs[i][0]) + (weightsSteer[6] * inputs[i][1]) + (weightsSteer[7] * inputs[i][2]) + (weightsSteer[8] * inputs[i][3]) + (weightsSteer[9] * inputs[i][4]) + thetaSteer[1];

            double outIntern11 = FunctionTransfer(net1);
            double outIntern12 = FunctionTransfer(net2);

            double net3 = (weightsSteer[10] * outIntern11) + (weightsSteer[11] * outIntern12) + thetaSteer[2];
            double net4 = (weightsSteer[12] * outIntern11) + (weightsSteer[13] * outIntern12) + thetaSteer[3];
            double net5 = (weightsSteer[14] * outIntern11) + (weightsSteer[15] * outIntern12) + thetaSteer[4];

            double outIntern21 = FunctionTransfer(net3);
            double outIntern22 = FunctionTransfer(net4);
            double outIntern23 = FunctionTransfer(net5);

            double net6 = (weightsSteer[16] * outIntern21) + (weightsSteer[17] * outIntern22) + (weightsSteer[18] * outIntern23) + thetaSteer[5];
            double net7 = (weightsSteer[19] * outIntern21) + (weightsSteer[20] * outIntern22) + (weightsSteer[21] * outIntern23) + thetaSteer[6];

            double outExtern1 = FunctionTransfer(net6);
            double outExtern2 = FunctionTransfer(net7);

            double[] deltaks = new double[2];

            if ((inputs[i][1] > 1f && inputs[i][3] < 5f) || Mathf.Abs((float)(inputs[i][0] - inputs[i][2])) < EPSILON)
            {
                // acelerar
                deltaks[0] = (0.6f - outExtern1) * DotFunctionTransfer(net3); // acelerar
                deltaks[1] = (0.4f - outExtern2) * DotFunctionTransfer(net4); // desacelerar
            }
            else if (inputs[i][1] < 1f || inputs[i][3] > 5f)
            {
                // desacelerar
                deltaks[0] = (0.4f - outExtern1) * DotFunctionTransfer(net3); // acelerar
                deltaks[1] = (0.6f - outExtern2) * DotFunctionTransfer(net4); // desacelerar
            }

            thetaSteer[5] = thetaSteer[5] + alpha * deltaks[0];
            thetaSteer[6] = thetaSteer[6] + alpha * deltaks[1];

            double[] Er = new double[3];
            Er[0] = (Mathf.Pow((float)deltaks[0], 2)) / 2f;
            Er[1] = (Mathf.Pow((float)deltaks[1], 2)) / 2f;
            double er = Er[0] + Er[1];

            if (er > EPSILON)
            {
                double[] delta1 = new double[3];
                delta1[0] = DotFunctionTransfer(net3) * (deltaks[0] * weightsSteer[16] + deltaks[1] * weightsSteer[19]);
                delta1[1] = DotFunctionTransfer(net4) * (deltaks[0] * weightsSteer[17] + deltaks[1] * weightsSteer[20]);
                delta1[2] = DotFunctionTransfer(net5) * (deltaks[0] * weightsSteer[18] + deltaks[1] * weightsSteer[21]);

                weightsSteer[10] = weightsSteer[10] + alpha * delta1[0] * outIntern11;
                weightsSteer[11] = weightsSteer[11] + alpha * delta1[0] * outIntern12;

                weightsSteer[12] = weightsSteer[12] + alpha * delta1[1] * outIntern11;
                weightsSteer[13] = weightsSteer[13] + alpha * delta1[1] * outIntern12;

                weightsSteer[14] = weightsSteer[14] + alpha * delta1[2] * outIntern11;
                weightsSteer[15] = weightsSteer[15] + alpha * delta1[2] * outIntern12;

                thetaSteer[2] = thetaSteer[2] + alpha * delta1[0];
                thetaSteer[3] = thetaSteer[3] + alpha * delta1[1];
                thetaSteer[4] = thetaSteer[4] + alpha * delta1[2];

                double[] delta0 = new double[2];
                delta0[0] = DotFunctionTransfer(net1) * (delta1[0] * weightsSteer[8] + delta1[1] * weightsSteer[10] + delta1[2] * weightsSteer[12]);
                delta0[1] = DotFunctionTransfer(net2) * (delta1[0] * weightsSteer[9] + delta1[1] * weightsSteer[11] + delta1[2] * weightsSteer[13]);

                weightsSteer[0] = weightsSteer[0] + alpha * delta0[0] * inputs[i][0];
                weightsSteer[1] = weightsSteer[1] + alpha * delta0[0] * inputs[i][1];
                weightsSteer[2] = weightsSteer[2] + alpha * delta0[0] * inputs[i][2];
                weightsSteer[3] = weightsSteer[3] + alpha * delta0[0] * inputs[i][3];
                weightsSteer[4] = weightsSteer[4] + alpha * delta0[0] * inputs[i][4];

                weightsSteer[5] = weightsSteer[5] + alpha * delta0[1] * inputs[i][0];
                weightsSteer[6] = weightsSteer[6] + alpha * delta0[1] * inputs[i][1];
                weightsSteer[7] = weightsSteer[7] + alpha * delta0[1] * inputs[i][2];
                weightsSteer[8] = weightsSteer[8] + alpha * delta0[1] * inputs[i][3];
                weightsSteer[9] = weightsSteer[9] + alpha * delta0[1] * inputs[i][4];

                thetaSteer[0] = thetaSteer[0] + alpha * delta0[0];
                thetaSteer[1] = thetaSteer[1] + alpha * delta0[1];
            }
        }

        return weightsSteer;
    }

    private double FunctionTransfer(double x)
    {
        double f = (1) / (1 + Mathf.Exp((float)-x));
        return f;
    }

    private double DotFunctionTransfer(double x)
    {
        double f_ = Mathf.Exp((float)-x) / Mathf.Pow((1 + Mathf.Exp((float)-x)), 2);
        return f_;
    }
}
