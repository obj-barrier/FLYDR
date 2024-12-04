using System.Collections.Generic;
using UnityEngine;

public class MapGenerator : MonoBehaviour
{
    public float safeDist;
    public GameObject drone;
    public GameObject goal;
    public GameObject map;

    public int width = 10;
    public int height = 10;
    public float roomSize = 1600f;
    public int tileNum = 15;
    readonly float xOffset = Mathf.Sqrt(3) / 2f;
    readonly float zOffset = 0.75f;

    public GameObject wall;
    public GameObject gate;
    public GameObject tunnel;
    public GameObject city;
    public GameObject goalCity;

    public GameObject indicator;
    public GameObject mainCamera;

    private bool[,] seen;
    private bool[,] renderC;
    private int count = 0;
    private int[] endP;

    private bool[,] hasRoom;
    private bool[,] hasCity;

    private readonly PathFinder pathFinder = new();
    private List<Vector3> path = new();
    private List<GameObject> cities = new();

    int Row()
    {
        int x = Random.Range(0, 2);
        return x;
    }

    Vector3 SpwnPos(int x, int z)
    {
        float xPos = roomSize * x * xOffset;
        if (z % 2 == 0)
        {
            xPos += roomSize * (xOffset / 2);
        }
        Vector3 SP = new(xPos, 0f, roomSize * z * zOffset);
        return SP;
    }

    void Dfs(List<List<int>> tileList, int x, int z)
    {
        seen[x, z] = true;
        count++;

        if (z % 2 == 0)
        {
            for (int i = x; i <= x + 1; i++)
            {
                for (int j = z - 1; j <= z + 1; j++)
                {
                    if (i == x && j == z)
                    {
                        int i_ = x - 1;
                        if (i_ >= 0 & j >= 0 & i_ < width & j < height)
                        {
                            renderC[i_, j] = true;
                            if (!seen[i_, j] && tileList[i_][j] == 0)
                            {
                                Dfs(tileList, i_, j);
                            }
                        }
                    }
                    else
                    {
                        if (i >= 0 & j >= 0 & i < width & j < height)
                        {
                            renderC[i, j] = true;
                            if (!seen[i, j] && tileList[i][j] == 0)
                            {
                                Dfs(tileList, i, j);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            for (int i = x - 1; i <= x; i++)
            {
                for (int j = z - 1; j <= z + 1; j++)
                {
                    if (i == x && j == z)
                    {
                        int i_ = x + 1;
                        if (i_ >= 0 & j >= 0 & i_ < width & j < height)
                        {
                            renderC[i_, j] = true;
                            if (!seen[i_, j] && tileList[i_][j] == 0)
                            {
                                Dfs(tileList, i_, j);
                            }
                        }
                    }
                    else
                    {
                        if (i >= 0 & j >= 0 & i < width & j < height)
                        {
                            renderC[i, j] = true;
                            if (!seen[i, j] && tileList[i][j] == 0)
                            {
                                Dfs(tileList, i, j);
                            }
                        }
                    }
                }
            }
        }
    }

    void GenRoom(int x, int z)
    {
        hasRoom[x, z] = true;

        int[] stru = new int[6];
        int[,] delta;
        if (z % 2 == 0)
        {
            int[,] tmpDelta = new int[2, 6] { { 1, 1, 1, 0,-1, 0 },
                                              { 1, 0,-1,-1, 0, 1 } };
            delta = tmpDelta;
        }
        else
        {
            int[,] tmpDelta = new int[2, 6] { { 0, 1, 0,-1,-1,-1 },
                                              { 1, 0,-1,-1, 0, 1 } };
            delta = tmpDelta;
        }

        for (int i = 0; i < 6; i++)
        {
            int x1 = x + delta[0, i];
            int z1 = z + delta[1, i];
            if (x1 >= 0 & z1 >= 0 & x1 < width & z1 < height)
            {
                if (seen[x1, z1])
                {
                    if (hasRoom[x1, z1])
                    {
                        stru[i] = 1;
                    }
                    else
                    {
                        stru[i] = 2;
                    }
                }
            }
        }

        Vector3 center = SpwnPos(x, z);
        float[,] deltaPos = new float[2, 6] { { 257, 514, 257,-257,-514,-257 },
                                              { 445,   0,-445,-445,   0, 445 } };
        for (int i = 0; i < 6; i++)
        {
            Vector3 pos1 = center + new Vector3(deltaPos[0, i], 0f, deltaPos[1, i]);
            Vector3 pos2 = center + 1.2f * new Vector3(deltaPos[0, i], 0f, deltaPos[1, i]);
            Quaternion rot = Quaternion.AngleAxis(30 + i * 60, Vector3.up);
            switch (stru[i])
            {
                case 2:
                    Instantiate(tunnel, pos2, rot, map.transform);
                    Instantiate(gate, pos1, rot, map.transform);
                    break;
                case 1:
                    Instantiate(gate, pos1, rot, map.transform);
                    break;
                default:
                    Instantiate(wall, pos1, rot, map.transform);
                    break;
            }
        }
    }
    
    void Awake()
    {
        pathFinder.SetSafeDist(safeDist);

        var tileList = new List<List<int>>();
        seen = new bool[width, height];
        renderC = new bool[width, height];
        hasRoom = new bool[width, height];
        hasCity = new bool[width, height];
        for (int x = 0; x < width; x++)
        {
            tileList.Add(new List<int>());
            for (int z = 0; z < height; z++)
            {
                tileList[x].Add(Row());
                renderC[x, z] = false;
                seen[x, z] = false;
                hasRoom[x, z] = false;
                hasCity[x, z] = false;
            }
        }

        int[] startP = new int[2];
        startP[0] = Random.Range(1, width / 2);
        startP[1] = Random.Range(1, height / 2);
        tileList[startP[0]][startP[1]] = 0;

        int safty = 0;
        Dfs(tileList, startP[0], startP[1]);
        while (count < tileNum && safty < 100)
        {
            count = 0;
            safty++;
            startP[0] = Random.Range(1, width);
            startP[1] = Random.Range(1, height);
            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < height; z++)
                {
                    seen[x, z] = false;
                }
            }
            Dfs(tileList, startP[0], startP[1]);
        }
        Debug.Log("start point:" + startP[0] + "," + startP[1]);
        Vector3 startPos = SpwnPos(startP[0], startP[1]) + safeDist * Vector3.up;
        Vector3 cameraOffset = mainCamera.transform.position - drone.transform.position;
        drone.transform.position = startPos;
        mainCamera.transform.position = startPos + cameraOffset;

        endP = new int[2];
        endP[0] = Random.Range(1, width);
        endP[1] = Random.Range(1, height);

        safty = 0;
        float dis_sq = (startP[0] - endP[0]) * (startP[0] - endP[0]) + (startP[1] - endP[1]) * (startP[1] - endP[1]);
        while (!seen[endP[0], endP[1]] || dis_sq < 25)
        {
            safty++;
            endP[0] = Random.Range(1, width);
            endP[1] = Random.Range(1, height);
            dis_sq = (startP[0] - endP[0]) * (startP[0] - endP[0]) + (startP[1] - endP[1]) * (startP[1] - endP[1]);
            if (safty > 10000)
            {
                Debug.Log("Failed to generate endpoint!");
                break;
            }
        }
        Debug.Log("end point:" + endP[0] + "," + endP[1]);
        Vector3 endPos = SpwnPos(endP[0], endP[1]) + safeDist * Vector3.up;
        goal.transform.position = SpwnPos(endP[0], endP[1]) + safeDist * Vector3.up;

        //actually generate the map
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < height; z++)
            {
                if (seen[x, z])
                {
                    GenRoom(x, z);
                    pathFinder.AddPoint(SpwnPos(x, z) + safeDist * Vector3.up, 0);
                    //GameObject hex = Instantiate(room, SpwnPos(x, z), Quaternion.AngleAxis(30f, Vector3.up), map.transform);
                    //hex.name = "Hex_" + x + "_" + z;
                } /*
                else
                {
                    GameObject hex = Instantiate(hexprefab[tileList[x][z]], spwnPos(x, z), Quaternion.identity);
                    hex.name = "Hex_" + x + "_" + z;
                } */
            }
        }
        /*foreach (PathFinder.Vertex v in pathFinder.vertices)
        {
            foreach (int i in v.link)
            {
                Debug.DrawLine(v.position, pathFinder.vertices[i].position, Color.green, 99999);
            }
        }*/
        path = pathFinder.FindPath(startPos, endPos, true);
    }

    // Update is called once per frame 每帧运行一次
    void Update()
    {
        int z = (int)(drone.transform.position.z / roomSize / zOffset + 0.5f);
        float xPos = drone.transform.position.x;
        if (z % 2 == 0)
        {
            xPos -= roomSize * (xOffset / 2);
        }
        int x = (int)(xPos / roomSize / xOffset + 0.5f);
        indicator.transform.position = SpwnPos(x, z) + 600f * Vector3.up;

        bool found = false;
        for (int i = 1; i < path.Count; i++)
        {
            if (Mathf.Abs(indicator.transform.position.x - path[i].x) < safeDist &&
                Mathf.Abs(indicator.transform.position.z - path[i].z) < safeDist)
            {
                found = true;
                goal.transform.position = path[i] + (path[i - 1] - path[i]) * 0.6f;
                break;
            }
        }
        if (!found)
        {
            goal.transform.position = path[0];
        }

        int[,] delta1, delta2;
        if (z % 2 == 0)
        {
            delta1 = new int[2, 6] { { 1, 1, 1, 0, -1, 0 },
                                     { 1, 0,-1,-1, 0, 1 } };
            delta2 = new int[2, 12] { { 0, 1, 2, 2, 2, 1, 0,-1,-1,-2,-1,-1 },
                                      { 2, 2, 1, 0,-1,-2,-2,-2,-1, 0, 1, 2 } };
        }
        else
        {
            delta1 = new int[2, 6]{ { 0, 1, 0,-1,-1,-1 },
                                     { 1, 0,-1,-1, 0, 1 } };
            delta2 = new int[2, 12] { { 0, 1, 1, 2, 1, 1, 0,-1,-2,-2,-2,-1 },
                                      { 2, 2, 1, 0,-1,-2,-2,-2,-1, 0, 1, 2 } };
        }
        for (int i = 0; i < 6; i++)
        {
            int x1 = x + delta1[0, i], z1 = z + delta1[1, i];
            if (x1 >= 0 & z1 >= 0 & x1 < width & z1 < height)
            {
                if (!hasCity[x1, z1] && seen[x1, z1])
                {
                    hasCity[x1, z1] = true;
                    if (x1 == endP[0] && z1 == endP[1])
                    {
                        cities.Add(Instantiate(goalCity, SpwnPos(x1, z1),
                            Quaternion.AngleAxis(Random.Range(0f, 360f), Vector3.up), map.transform));
                    }
                    else
                    {
                        cities.Add(Instantiate(city, SpwnPos(x1, z1),
                            Quaternion.AngleAxis(Random.Range(0f, 360f), Vector3.up), map.transform));
                    }
                }
            }
        }
        for (int i = 0; i < 12; i++)
        {
            int x2 = x + delta2[0, i], z2 = z + delta2[1, i];
            if (x2 >= 0 & z2 >= 0 & x2 < width & z2 < height)
            {
                if (hasCity[x2, z2])
                {
                    foreach (GameObject g in cities)
                    {
                        if (Vector3.Distance(SpwnPos(x2, z2), g.transform.position) < safeDist)
                        {
                            g.SetActive(false);
                        }
                    }
                }
            }
        }
    }
}
