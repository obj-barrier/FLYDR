using System.Collections.Generic;
using UnityEngine;

public class PathFinder
{
    private float safeDist;

    public void SetSafeDist(float safeDist)
    {
        this.safeDist = safeDist;
    }

    public class Vertex
    {
        public int index;
        public Vector3 position;
        public List<int> link;

        public Vertex(int index, Vector3 position)
        {
            this.index = index;
            this.position = position;
            link = new List<int>();
        }
    } public List<Vertex> vertices = new();

    public void AddPoint(Vector3 position, int mode)
    {
        if (mode == 0)
        {
            Vertex v = new(vertices.Count, position);
            for (int i = 0; i < vertices.Count; i++)
            {
                if (Vector3.Distance(position, vertices[i].position) < 2000f)
                {
                    v.link.Add(i);
                    vertices[i].link.Add(v.index);
                }
            }
            vertices.Add(v);
        }
        else
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                if (Vector3.Distance(position, vertices[i].position) < safeDist)
                {
                    return;
                }
            }

            Vertex v = new(vertices.Count, position);
            for (int i = 0; i < vertices.Count; i++)
            {
                if (DroneController.HasSafeLink(position, vertices[i].position, 0.8f * safeDist))
                {
                    v.link.Add(i);
                    vertices[i].link.Add(v.index);
                }
            }
            vertices.Add(v);
        }
    }

    int FindNearest(Vector3 point)
    {
        int minIdx = -1;
        float minDist = float.MaxValue;
        foreach (Vertex v in vertices)
        {
            float dist = Vector3.Distance(point, v.position);
            if (dist < minDist)
            {
                minIdx = v.index;
                minDist = dist;
            }
        }
        return minIdx;
    }

    public Vector3 FindPath(Vector3 start, Vector3 goal)
    {
        int s = FindNearest(start);
        int t = FindNearest(goal);

        int n = vertices.Count;
        List<bool> found = new(new bool[n]);
        List<float> dist = new(new float[n]);
        List<int> last = new(new int[n]);
        for (int i = 0; i < n; i++)
        {
            dist[i] = float.MaxValue;
        }

        foreach (int i in vertices[s].link)
        {
            dist[i] = Vector3.Distance(vertices[s].position, vertices[i].position);
            last[i] = s;
        }
        dist[s] = 0f;
        found[s] = true;

        while (!found[t])
        {
            float min = float.MaxValue;
            int subStart = 0;
            for (int i = 0; i < n; i++)
            {
                if (!found[i] && dist[i] < min)
                {
                    min = dist[i];
                    subStart = i;
                }
            }
            found[subStart] = true;

            foreach (int i in vertices[subStart].link)
            {
                float tmpDist = Vector3.Distance(vertices[subStart].position, vertices[i].position);
                if (!found[i] && dist[subStart] + tmpDist < dist[i])
                {
                    dist[i] = dist[subStart] + tmpDist;
                    last[i] = subStart;
                }
            }
        }

        int now = t;
        while (last[now] != s && !DroneController.HasSafeLink(vertices[s].position, vertices[now].position, safeDist))
        {
            Debug.DrawLine(vertices[last[now]].position, vertices[now].position, Color.magenta);
            now = last[now];
        }
        return vertices[now].position;
    }

    public List<Vector3> FindPath(Vector3 start, Vector3 goal, bool flag)
    {
        int s = FindNearest(start);
        int t = FindNearest(goal);

        int n = vertices.Count;
        List<bool> found = new(new bool[n]);
        List<float> dist = new(new float[n]);
        List<int> last = new(new int[n]);
        for (int i = 0; i < n; i++)
        {
            dist[i] = float.MaxValue;
        }

        foreach (int i in vertices[s].link)
        {
            dist[i] = Vector3.Distance(vertices[s].position, vertices[i].position);
            last[i] = s;
        }
        dist[s] = 0f;
        found[s] = true;

        while (!found[t])
        {
            float min = float.MaxValue;
            int subStart = 0;
            for (int i = 0; i < n; i++)
            {
                if (!found[i] && dist[i] < min)
                {
                    min = dist[i];
                    subStart = i;
                }
            }
            found[subStart] = true;

            foreach (int i in vertices[subStart].link)
            {
                float tmpDist = Vector3.Distance(vertices[subStart].position, vertices[i].position);
                if (!found[i] && dist[subStart] + tmpDist < dist[i])
                {
                    dist[i] = dist[subStart] + tmpDist;
                    last[i] = subStart;
                }
            }
        }

        List<Vector3> path = new();
        int now = t;
        while (now != s)
        {
            path.Add(vertices[now].position);
            Debug.DrawLine(vertices[last[now]].position,
                vertices[now].position, Color.magenta, 99999);
            now = last[now];
        }
        path.Add(vertices[now].position);
        return path;
    }
}
