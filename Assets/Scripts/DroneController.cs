using System.Collections.Generic;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    public GameObject goal;
    public GameObject markSubgoal;
    public GameObject markSubgoalBest;
    public bool debugDrawPath, debugDrawSafepoints, debugDrawExits, debugDrawScanlines;
    public bool debugDrawNetwork;
    public float safeDist, accel, maxSpeed;

    private Vector3 velocity;
    private Vector3 lastGoal;
    private List<Vector3> scanning = new(), scanned = new();

    private PathFinder pathFinder = new();

    public class Path
    {
        public Vector3 position;
        public int scannedIdx;
        public Path(Vector3 position, int scannedIdx)
        {
            this.position = position;
            this.scannedIdx = scannedIdx;
        }
    } private readonly List<Path> path = new();

    public class Exit
    {
        public bool deleted;
        public int index;
        public Vector3 endpointLeft;
        public Vector3 endpointRight;
        public bool isLockedLeft;
        public bool isLockedRight;
        public Vector3 normalStart;
        public Vector3 normalEnd;

        public Exit(int index, Vector3 left, Vector3 right, float safeDist)
        {
            deleted = false;
            this.index = index;
            endpointLeft = left;
            endpointRight = right;
            isLockedLeft = false;
            isLockedRight = false;
            UpdateNormal(safeDist);
        }

        public void Delete()
        {
            deleted = true;
        }
        public void UpdateLeft(Vector3 point)
        {
            endpointLeft = point;
        }
        public void UpdateRight(Vector3 point)
        {
            endpointRight = point;
        }
        public void LockLeft()
        {
            isLockedLeft = true;
            if (isLockedRight)
            {
                Delete();
            }
        }
        public void LockRight()
        {
            isLockedRight = true;
            if (isLockedLeft)
            {
                Delete();
            }
        }
        public void UpdateNormal(float safeDist)
        {
            normalStart = (endpointLeft + endpointRight) / 2f;
            normalEnd = normalStart + Quaternion.AngleAxis(-90, Vector3.up) *
                        (endpointRight - endpointLeft).normalized * safeDist;
        }
    } private List<Exit> exits = new();

    void SplitExit(Exit @exit, Vector3 point)
    {
        exits.Add(new Exit(exits.Count, point, exit.endpointRight, safeDist));
        exit.UpdateRight(point);
    }

    bool IsScanned(Vector3 point)
    {
        foreach (Vector3 p in scanned)
        {
            if (Vector3.Distance(point, p) < safeDist)
            {
                return true;
            }
        }
        return false;
    }

    bool HasLock(int index, Vector3 point, int orientation)
    {
        if (orientation < 0)
        {
            for (int i = 0; i < exits.Count; i++)
            {
                if (i != index && !exits[i].deleted)
                {
                    double dist1 = Vector3.Distance(exits[index].endpointRight, exits[i].endpointLeft);
                    double dist2 = Vector3.Distance(point, exits[i].endpointLeft);
                    if (dist1 > safeDist && dist2 < safeDist)
                    {
                        exits[i].LockLeft();
                        return true;
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < exits.Count; i++)
            {
                if (i != index && !exits[i].deleted)
                {
                    double dist1 = Vector3.Distance(exits[index].endpointLeft, exits[i].endpointRight);
                    double dist2 = Vector3.Distance(point, exits[i].endpointRight);
                    if (dist1 > safeDist && dist2 < safeDist)
                    {
                        exits[i].LockRight();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void Scan(Exit exit, Vector3 start, int orientation) // 从出口的端点开始向内部扫描
    {
        Vector3 hitLast = start; // 记录上次的扫描点
        for (int i = 0; i < 1000; i++) // 从每个端点出发最多进行1000次扫描
        {
            // 设定射线方向
            Vector3 direction = Quaternion.AngleAxis(orientation, transform.up) * (hitLast - transform.position);
            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, 1000f)) // 发射射线
            {
                if (Vector3.Distance(hit.point, hitLast) > 2f * safeDist) // 如果新扫描点离上次的扫描点足够远
                {
                    // 则先尝试细分扫描，在上次的扫描点附近找新方向
                    direction = hitLast + (hit.point - hitLast).normalized * (safeDist / 10f) - transform.position;
                    if (Physics.Raycast(transform.position, direction, out RaycastHit hit2, 1000f)) // 再次发射射线
                    {
                        hit = hit2; // 更新扫描点
                    }
                }

                // 如果新扫描点离上一个记录的扫描点足够远
                if (scanning.Count == 0 || Vector3.Distance(hit.point, scanning[^1]) > safeDist)
                {
                    scanning.Add(hit.point); // 则记录新扫描点
                }

                if (Vector3.Distance(hit.point, hitLast) <= 2f * safeDist) // 如果新扫描点离上次的扫描点足够近
                {
                    if (orientation < 0) // 如果当前在逆时针扫描（即从出口的右端点开始扫描）
                    {
                        if (Vector3.Distance(hit.point, exit.endpointLeft) > 2f * safeDist) // 如果离左端点足够远
                        {
                            if (HasLock(exit.index, hit.point, -1))
                            {
                                exit.LockRight();
                                /*if (终点不在两出口夹角范围内)
                                {
                                    删除两出口;
                                }*/
                                break;
                            }
                            exit.UpdateRight(hit.point); // 则更新右端点
                        }
                        else // 否则离左右端点都近
                        {
                            exit.Delete(); // 删除这个出口
                            break;
                        }
                    }
                    else // ↓顺时针扫描则与右端点做相同比较
                    {
                        if (Vector3.Distance(hit.point, exit.endpointRight) > 2f * safeDist)
                        {
                            if (HasLock(exit.index, hit.point, 1))
                            {
                                exit.LockLeft();
                                break;
                            }
                            exit.UpdateLeft(hit.point); // 则更新左端点
                        }
                        else
                        {
                            exit.Delete();
                            break;
                        }
                    } // ↑顺时针扫描则与右端点做相同比较
                }
                else // 如果新扫描点距上一个扫描点足够远
                {
                    if (orientation > 0 && !exit.isLockedRight) // 且正在顺时针扫描
                    {
                        if (Vector3.Distance(hit.point, exit.endpointRight) > 2f * safeDist) // 且新扫描点离右端点也远
                        {
                            float distLeft = Vector3.Distance(transform.position, exit.endpointLeft);
                            float distRight = Vector3.Distance(transform.position, exit.endpointRight);
                            if (hit.distance > distLeft || hit.distance > distRight)
                            {
                                Vector3 droneToHit = hit.point - transform.position;
                                Vector3 droneToRight = exit.endpointRight - transform.position;
                                if (Vector3.SignedAngle(droneToHit, droneToRight, Vector3.up) > 0) // 且新扫描点在左右端点之间
                                {
                                    if (!IsScanned(hit.point)) // 且新扫描点附近未被扫描过
                                    {
                                        SplitExit(exit, hit.point); // 则在扫描点处拆分出口
                                    }
                                }
                            }
                        }
                    }
                    break; // 无论在哪个方向扫描，无论是否新建了出口，都停止扫描
                }
                hitLast = hit.point; // 更新上次的扫描点
            }
        }
        exit.UpdateNormal(safeDist);
    }

    void NetworkScan()
    {
        Vector3 directionStart = Quaternion.AngleAxis(Time.frameCount * 6 % 360, Vector3.up) * Vector3.forward;
        Physics.Raycast(transform.position, directionStart, out RaycastHit hit, 1000f);
        Vector3 hitLast = hit.point; // 也作为上次扫描的点
        for (int i = 1; i < 6; i++) // 继续进行完整扫描
        {
            Vector3 direction = Quaternion.AngleAxis(i, transform.up) * directionStart; // 设定射线方向
            if (Physics.Raycast(transform.position, direction, out hit, 1000f)) // 发射射线
            {
                if (Vector3.Distance(hit.point, hitLast) > 2f * safeDist) // 如果新扫描点离上次扫描的点足够远
                {
                    float distLast = Vector3.Distance(transform.position, hitLast);
                    float dist = Vector3.Distance(transform.position, hit.point);
                    if (distLast < dist)
                    {
                        Vector3 safePoint = FindSafePoint(hitLast, 1);
                        if (HasSafeLink(transform.position, safePoint, safeDist))
                        {
                            pathFinder.AddPoint(safePoint, 1);
                        }
                    }
                    else
                    {
                        Vector3 safePoint = FindSafePoint(hit.point, 1);
                        if (HasSafeLink(transform.position, safePoint, safeDist))
                        {
                            pathFinder.AddPoint(safePoint, 1);
                        }
                    }
                }
                hitLast = hit.point; // 更新上次扫描的点
            }
        }
    }

    void InitScan() // 第一次扫描
    {
        // 先向正前方发射射线
        Physics.Raycast(transform.position, transform.forward, out RaycastHit hit, 1000f);
        scanning.Add(hit.point); // 记录第一个扫描点
        Vector3 hitLast = hit.point; // 也作为上次扫描的点
        for (int i = 1; i < 360; i++) // 继续进行完整扫描
        {
            Vector3 direction = Quaternion.AngleAxis(i, transform.up) * transform.forward; // 设定射线方向
            if (Physics.Raycast(transform.position, direction, out hit, 1000f)) // 发射射线
            {
                if (Vector3.Distance(hit.point, scanning[^1]) > safeDist) // 如果新扫描点离上一个记录的扫描点足够远
                {
                    scanning.Add(hit.point); // 则记录新扫描点
                }
                if (Vector3.Distance(hit.point, hitLast) > 2f * safeDist) // 如果新扫描点离上次扫描的点足够远
                {
                    float distLast = Vector3.Distance(transform.position, hitLast);
                    float dist = Vector3.Distance(transform.position, hit.point);
                    if (distLast < dist)
                    {
                        pathFinder.AddPoint(FindSafePoint(hitLast, 1), 1);
                    }
                    else
                    {
                        pathFinder.AddPoint(FindSafePoint(hit.point, 1), 1);
                    }

                    exits.Add(new Exit(exits.Count, hitLast, hit.point, safeDist)); // 则在两点间新建出口
                }
                hitLast = hit.point; // 更新上次扫描的点
            }
        }
        scanned.AddRange(scanning); // 永久保存第一轮扫描的点
        path.Add(new Path(transform.position, scanned.Count - 1)); // 保存路径起点位置
    }

    bool IsInSight(Vector3 target) // 判断两点之间是否有直接链接
    {
        Vector3 direction = target - transform.position; // 确定方向
        if (Physics.Raycast(transform.position, direction, out RaycastHit hit, 1000f)) // 尝试直接连接
        {
            // 如果射线被阻挡的距离接近接近目标位置，则认为有链接
            return hit.distance > Vector3.Distance(target, transform.position) - safeDist / 10f;
        }
        return true; // 如果射线未被阻挡，也判定为有链接
    }

    public static bool HasSafeLink(Vector3 start, Vector3 end, float safeDist) // 判断两点之间是否有足够飞行空间
    {
        Vector3 direction = end - start; // 确定方向
        if (Physics.Raycast(start, direction, out RaycastHit hit1, 1000f)) // 尝试直接连接
        {
            if (hit1.distance < Vector3.Distance(start, end) + 0.5f * safeDist) // 如果直接连接被阻挡
            {
                return false; // 则没有足够飞行空间
            }
        }
        // ↓对左右两侧各半个翼展的距离进行相同检测
        Vector3 start2 = start + (Quaternion.AngleAxis(90f, Vector3.up) * direction).normalized * (0.5f * safeDist);
        Vector3 end2 = end + (Quaternion.AngleAxis(90f, Vector3.up) * direction).normalized * (0.5f * safeDist);
        if (Physics.Raycast(start2, direction, out RaycastHit hit2, 1000f))
        {
            if (hit2.distance < Vector3.Distance(start2, end2) - 1)
            {
                return false;
            }
        }
        Vector3 start3 = start + (Quaternion.AngleAxis(-90f, Vector3.up) * direction).normalized * (0.5f * safeDist);
        Vector3 end3 = end + (Quaternion.AngleAxis(90f, Vector3.up) * direction).normalized * (0.5f * safeDist);
        if (Physics.Raycast(start3, direction, out RaycastHit hit3, 1000f))
        {
            if (hit3.distance < Vector3.Distance(start3, end3) - 1)
            {
                return false;
            }
        } // ↑对左右两侧各半个翼展的距离进行相同检测

        return true; // 左右空间都足够大，可以安全飞行
    }

    Vector3 FindSafePoint(Vector3 point, int orientation) // 找某点的安全点，优先在输入的方向找
    {
        // ↓以目标点为圆心，安全距离为半径作圆，求从当前位置出发的左右2个切点坐标↓
        float dist = Vector3.Distance(transform.position, point);
        float ang = Mathf.Acos(safeDist / dist) * Mathf.Rad2Deg;
        Vector3 testPoint = point + Quaternion.AngleAxis(-orientation * ang, Vector3.up) *
            ((transform.position - point).normalized * safeDist);
        if (!float.IsNaN(testPoint.x))
        {
            if (HasSafeLink(transform.position, testPoint, safeDist))
            {
                return testPoint;
            }
        }

        testPoint = point + Quaternion.AngleAxis(orientation * ang, Vector3.up) *
            ((transform.position - point).normalized * safeDist);
        if (!float.IsNaN(testPoint.x))
        {
            if (HasSafeLink(transform.position, testPoint, safeDist))
            {
                return testPoint;
            }
        }
        // ↑以目标点为圆心，安全距离为半径作圆，求从当前位置出发的左右2个切点坐标↑

        // 如果左右2个切点都不安全，则尝试直接前往目标点，但在安全距离处停下
        return point + (transform.position - point).normalized * safeDist;
    }

    void Fly(Vector3 target) // 执行飞行（给自己加速度）
    {
        velocity += (target - transform.position - velocity).normalized * (accel / 60f); // 施加朝向目标方向的加速度，持续1帧
        if (velocity.magnitude > maxSpeed) // 如果加后的速度超过最高速度限制
        {
            velocity = velocity.normalized * maxSpeed; // 则方向不变，大小减少为最高速度
        }
        velocity.y = 0f;
        transform.forward = velocity; // 机头朝向新的速度方向
        transform.position += velocity / 60f; // 计算并应用这1帧内的位移

    }

    // Start is called before the first frame update 游戏开始前运行一次
    void Start()
    {
        pathFinder.SetSafeDist(safeDist);
        InitScan(); // 初始进行360度扫描
    }

    // Update is called once per frame 每帧运行一次
    void Update()
    {
        if (Vector3.Distance(lastGoal, goal.transform.position) > safeDist)
        {
            scanning = new();
            scanned = new();
            pathFinder = new();
            exits = new();
            //transform.position.Set(transform.position.x, 50f, transform.position.z);
            //transform.eulerAngles.Set(0f, transform.eulerAngles.y, 0f);
            InitScan();
        }
        lastGoal = goal.transform.position;

        int cnt = exits.Count; // 记录出口数量

        if (debugDrawExits) // 如果开启了出口显示
        {
            for (int i = 0; i < cnt; i++) // 遍历每个已发现的出口
            {
                if (!exits[i].deleted) // 如果第i个出口没有被删除
                {
                    // 则用红线画出它左右端点的连线，持续0.1秒实现加粗效果
                    Debug.DrawLine(exits[i].endpointLeft, exits[i].endpointRight, Color.red, 0.1f);
                    // 并用绿线画出它的法线，持续0.1秒实现加粗效果
                    Debug.DrawLine(exits[i].normalStart, exits[i].normalEnd, Color.green, 0.1f);
                }
            }
        }

        if (debugDrawNetwork)
        {
            foreach (PathFinder.Vertex v in pathFinder.vertices)
            {
                foreach (int i in v.link)
                {
                    if (Random.value < 2f / v.link.Count)
                    {
                        Debug.DrawLine(v.position + 10f * Vector3.down,
                            pathFinder.vertices[i].position + 10f * Vector3.down, Color.green);
                    }
                }
            }
        }

        NetworkScan();

        scanning.Clear(); // 重新开始记录本次寻路扫描过的点（清空上一次的点）
        for (int i = 0; i < cnt; i++)
        {
            if (!exits[i].deleted) // 如果第i个出口没有被删除
            {
                if (!exits[i].isLockedLeft && IsInSight(exits[i].endpointLeft)) // 如果当前能看到出口的左端点
                {
                    Scan(exits[i], exits[i].endpointLeft, 1); // 则从左端点开始顺时针扫描
                }
                if (!exits[i].isLockedRight && IsInSight(exits[i].endpointRight)) // 如果当前能看到出口的右端点
                {
                    Scan(exits[i], exits[i].endpointRight, -1); // 则从右端点开始逆时针扫描
                }
            }
        }

        if (Time.frameCount % 60 == 0) // 每秒只执行1次
        {
            scanned.AddRange(scanning); // 保存本次扫描的所有点
            path.Add(new Path(transform.position, scanned.Count - 1)); // 记录当前所在位置，以及哪些点是从此处扫描到的
        }

        if (debugDrawPath) // 如果开启了路径显示
        {
            for (int i = 1; i < path.Count; i++) // 遍历之前走过的路径
            {
                Debug.DrawLine(path[i - 1].position, path[i].position, Color.white); // 用白线连接路径
            }
        }

        if (debugDrawScanlines)  // 如果开启了扫描线显示
        {
            int index = 0; // 第0个路径点对应最初的扫描点
            for (int i = 0; i < scanned.Count; i++) // 遍历所有扫描过的点
            {
                if (i > path[index].scannedIdx) // 如果新扫描点已经不属于当前路径点
                {
                    index++; // 则从切换到下一个路径点（否则还是从当前路径点开始画）
                }
                // 画出从路径点到扫描点的蓝线（画在地下实现半透明，画2根调整透明度）
                Debug.DrawLine(path[index].position + 10f * Vector3.down,
                    scanned[i] + 10f * Vector3.down, Color.blue, 0f, true);
                Debug.DrawLine(path[index].position + 10f * Vector3.down,
                    scanned[i] + 10f * Vector3.down, Color.blue, 0f, true);
            }
        }

        float minDist = float.MaxValue, minDistAbs = float.MaxValue; // 初始化最小距离
        Vector3 subGoalBest = new(), subGoalAbsBest = new(); // 初始化最佳目标
        for (int i = 0; i < cnt; i++) // 遍历每个已发现的出口
        {
            if (!exits[i].deleted) // 如果第i个出口没有被合并或删除
            {
                Vector3 subGoal = FindSafePoint(exits[i].endpointLeft, 1); // 计算它左端点对应的安全点，优先找端点右侧的
                if (debugDrawSafepoints) // 如果开启了安全点显示
                {
                    Instantiate(markSubgoal, subGoal, Quaternion.identity); // 则生成黄球标记
                }
                // 计算 当前位置→安全点→终点 的直线距离 ☆需要改进！
                float dist = Vector3.Distance(transform.position, subGoal) +
                    Vector3.Distance(subGoal, goal.transform.position);
                if (dist < minDistAbs) // 如果比当前记录的最小距离更小
                {
                    subGoalAbsBest = subGoal; // 则暂定此安全点为最佳目标
                    minDistAbs = dist; // 并刷新最小距离
                }
                if (HasSafeLink(transform.position, subGoal, safeDist)) // 如果能直达此安全点
                {
                    if (dist < minDist) // 如果比当前记录的最小距离更小
                    {
                        subGoalBest = subGoal; // 则暂定此安全点为最佳目标
                        minDist = dist; // 并刷新最小距离
                    }
                }
                subGoal = FindSafePoint(exits[i].endpointRight, -1); // ↓对此出口的右端点进行相同操作↓
                if (debugDrawSafepoints)
                {
                    Instantiate(markSubgoal, subGoal, Quaternion.identity);
                }
                dist = Vector3.Distance(transform.position, subGoal) +
                    Vector3.Distance(subGoal, goal.transform.position);
                if (dist < minDistAbs)
                {
                    subGoalAbsBest = subGoal;
                    minDistAbs = dist;
                }
                if (HasSafeLink(transform.position, subGoal, safeDist))
                {
                    if (dist < minDist)
                    {
                        subGoalBest = subGoal;
                        minDist = dist;
                    }
                } // ↑对此出口的右端点进行相同操作↑
            }
        }

        if (HasSafeLink(transform.position, goal.transform.position, safeDist)) // 如果已经能直达终点
        {
            if (Vector3.Distance(transform.position, goal.transform.position) < safeDist) // 如果已经到达终点
            {
                return; // 则直接结束寻路
            }
            Fly(goal.transform.position); // 否则直接飞向终点
            return; // 并结束本次寻路
        }
        if (subGoalBest == Vector3.zero)
        {
            subGoalBest = pathFinder.FindPath(transform.position, subGoalAbsBest);
        }

        if (debugDrawSafepoints) // 如果开启了安全点显示
        {
            Instantiate(markSubgoalBest, subGoalBest, Quaternion.identity); // 在最佳目标处生成红球标记
        }

        if (Vector3.Distance(transform.position, subGoalBest) < 0.5f * safeDist) // 如果当前已经到达最佳目标
        {
            Fly(transform.position + transform.forward * 1000f); // 则尝试继续向正前方飞行
        }
        else
        {
            Fly(subGoalBest); // 否则向最佳目标飞行
        }
    }
}
