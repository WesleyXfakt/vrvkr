using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Timers;
using System.Linq;

public class trafficAI : MonoBehaviour
{
    #region variables
    [Header("Speed values")]
    public float maxSpeed = 8;
    public float targetSpeed;
    public float currentSpeed;
    public float difference;
    public bool isBrakingForStopSign;
    [Header("Brake values")]
    public float targetBrakeValue;
    public float currentBrakeValue;
    public float relevantBrakeDistance;
    public bool frontRightSensorSpotted;
    public bool frontLeftSensorSpotted;
    [Header("Acceleration values")]
    public float currentAccelerationValue;
    public float targetAccelerationValue;

    public bool debugBreak = false;
    [HideInInspector]
    public bool hasApproachedPlayer = false;

    [HideInInspector]
    public int leftAngleDiff;
    [HideInInspector]
    public int rightAngleDiff;

    private bool brakeForStopSign;

    [Header("-- Wheel colliders")]
    [HideInInspector]
    public WheelCollider wheelFL;
    [HideInInspector]
    public WheelCollider wheelFR;
    [HideInInspector]
    public WheelCollider wheelBL;
    [HideInInspector]
    public WheelCollider wheelBR;

    [Header("-- Car visuals/lights")]
    [HideInInspector]
    public Texture2D textureNormal;
    [HideInInspector]
    public Texture2D textureBraking;
    [HideInInspector]
    public Texture2D textureHeadlightOn;
    [HideInInspector]
    public Renderer carRenderer;

    [Header("-- Sensors")]
    [HideInInspector]
    public float sensorLength;
    [HideInInspector]
    public Vector3 frontSensorPos;
    [HideInInspector]
    public float frontSideSensorPos;
    [HideInInspector]
    public float frontSensorAngle = 30;
    [HideInInspector]
    public float angledSensorLengthDiff;

    [Header("-- Pathfinding")]
    private List<Transform> nodes;
    [Header("-- The node we want the car to start at. Set to the nearest node in the path")]
    public int currentNode;
    // Set the path from the player when giving parking instructions
    public Transform path;
    [HideInInspector]
    public bool stopped;
    private bool turning;
    [HideInInspector]
    public bool deciding;

    [Header("-- Car variables")]
    [HideInInspector]
    public Vector3 centerOfMass;
    public float maxBrakeTorque = 150f;
    public float maxSteerAngle = 45f;
    public float turnSpeed;
    public float maxMotorTorque = 80f;

    public float initialMaxSpeed;
    private float targetSteerAngle = 0;

    [HideInInspector]
    public bool tutorialCar;
    [HideInInspector]
    public float buitenDeBebouwdeKomSpeed;

    public int roundedSpeed;

    [HideInInspector]
    public AudioSource car;
    [HideInInspector]
    public AudioClip honkSound;

    [HideInInspector]
    public bool isBraking = false;
    public bool approachingStopLine;

    [Header("-- Experimental")]
    //[HideInInspector]
    public bool pathLoops = true;
    private scenarioManager scenarioManager;
    private PedalScript ps;
    [HideInInspector]
    public Transform bufferedPath;
    private CarPooler cp;
    [HideInInspector]
    public DifficultyInformationIntersection.direction currentStreet;


    [Header("-- Audio")]
    AudioSource m_MyAudioSource;
    [HideInInspector]
    public float soundPitchDiff = 1;
    [HideInInspector]
    public float modifier;
    //Value from the slider, and it converts to volume level
    [HideInInspector]
    public float targetEngineVolume;
    public AudioClip crashSound;


    [HideInInspector]
    public bool playerHit;

    public GameObject lights;

    private Rigidbody rb;

    public float minimumDistance;

    public bool isTouchingCar;
    public float isTouchingCarTimer;

    private RigidbodyConstraints initialConstraints;
    private Coroutine disableConstraintCoroutine;
    private List<Coroutine> coroutinesToCancel = new List<Coroutine>();

    private List<CarWheel> carWheels;

    public GameObject ReasonForBraking;

    public int allowedWaitingTime = 30;
    private float timeStandingStill = 0;
    public int secondsStandingStill;
    public int excessSecondsStoodStill;
    private float excessSecFloat;
    public bool waitingToLong;
    public bool hasHonkedForStandingStillTooLong;
    public bool speedCheckPerformed;

    public Vector3 NWCorner;
    public Vector3 SECorner;
    private bool intersectionCornersPresent;

    public bool isPositionedOnIntersection;
    public bool hasCrashed;
    private PootManager pm;

    private bool spottedAtIntersectionCoroutineRunning;
    #endregion

    #region Start/Update
    private void Start()
    {
        ps = GetComponent<PedalScript>();

        // this only checks for the NW corner, but it will do for now
        intersectionCornersPresent = GameObject.Find("IntersectionCornerNW") == null ? false : true;

        if (intersectionCornersPresent)
        {
            NWCorner = GameObject.Find("IntersectionCornerNW").transform.position;
            SECorner = GameObject.Find("IntersectionCornerSE").transform.position;
        }

        carWheels = GetComponentsInChildren<CarWheel>().ToList();
        pm = GameObject.FindObjectOfType<PootManager>();
        LerpBrakeAmount();
        LerpAccelerationAmount();
        cp = GameObject.FindObjectOfType<CarPooler>();

        //StartCoroutine(printEverySecond());
        rb = GetComponent<Rigidbody>();
        //StartCoroutine(RoundSpeedToInt());
        // als ik deze uitzet rond hij niet meer af i guess
        //m_MyAudioSource = GetComponent<AudioSource>();
        //m_MyAudioSource.rolloffMode = AudioRolloffMode.Linear;



        // Set the centre of mass at the bottom of the car for more stability
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;

        // Make the list of nodes available locally
        //Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        //nodes = new List<Transform>();

        //for (int i = 0; i < pathTransforms.Length; i++)
        //{
        //    if (pathTransforms[i] != path.transform)
        //    {
        //        nodes.Add(pathTransforms[i]);
        //    }
        //}

        if (rb != null)
        {
            initialConstraints = rb.constraints;
        }

        UpdatePath();

        if (GameObject.Find("scenarioManager") != null)
        {
            scenarioManager = GameObject.Find("scenarioManager").GetComponent<scenarioManager>();

            if (scenarioManager.scenarioName == scenarioEnum.buitenDeBebouwdeKom)
            {
                maxBrakeTorque = 8000;

            }
        }
        initialMaxSpeed = maxSpeed;

    }

    private void Awake()
    {
        // Get the car's audiosource to play clips
        car = GetComponent<AudioSource>();
        honkSound = Resources.Load("audio/honk", typeof(AudioClip)) as AudioClip;
        if(car != null)
        {
            car.clip = honkSound;
        }
        crashSound = Resources.Load("audio/shortestDistanceCrash", typeof(AudioClip)) as AudioClip;
    }

    private void Update()
    {
        CheckIfPositionedOnIntersection();
        MeasureWaitingTime();
    }

    // Applied 60 times a second regardless of framerate. Behaviour is handled in the respective methods
    private void FixedUpdate()
    {
        CoachFeedbackWhenNotSpotted();
        LerpBrakeAmount();
        LerpAccelerationAmount();
        SetIsSpottedWhenWaitingForTraffic();
        Sensors();
        ApplySteer();
        MatchTargetSpeed();
        RespondToNode();
        CheckBrakeLights();
        ReactToPlayerSigns();
        LerpToSteerAngle();
        ResetWheelColliders();
        MidIntersectionFailsafe();
        ps.currentSpeed = currentSpeed;
        currentSpeed = Convert.ToInt32(rb.velocity.magnitude * 3.6f);
        ps.targetSpeed = targetSpeed;

        if (transform.position.y < -100)
        {
            Destroy(this.gameObject);
        }

        if (isTouchingCar)
        {
            IncrementCollidingTimer();
        }
        else
        {
            isTouchingCarTimer = 0;
        }

        if (brakeForStopSign)
        {
            DetermineSpeed(0f);
        }

        if (debugBreak)
        {
            targetSpeed = 0;
        }

        if (hasCrashed)
        {
            StartCoroutine(StartDrivingAfterBeingCrashed());
        }
    }
    #endregion

    #region Path methods
    public void UpdatePath()
    {
        // Make the list of nodes available locally
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
    }

    // When the agent reaches a node, increment the current node to follow the path and brake in case it's approaching a turn
    private void RespondToNode()
    {
        // When the car hits the node have it react
        if (Vector3.Distance(transform.position, nodes[currentNode].position) < 3f)
        {
            deciding = false;

            if (nodes[currentNode].CompareTag("Turn") && currentSpeed > 5)
            {
                turning = true;
            }
            else
            {
                turning = false;
            }

            if (nodes[currentNode].CompareTag("park"))
            {
                // Turn off engine
                StopCar();
                TurnOffRayCast();
            }

            // Take the chosen path from the decisionnode
            if (nodes[currentNode].CompareTag("decisionNode"))
            {
                deciding = true;

                // Path was set before car arrived
                if (bufferedPath != null)
                {
                    UpdatePath(bufferedPath);
                    bufferedPath = null;
                }
                else
                {
                    StopCar();
                    path = null;
                    StartCoroutine(CheckForNewPath(1));
                }
            }

            // StopLine nodes are located on stoplines. Used to make car stop at stopline regardless of distance
            for (int node = currentNode; node < currentNode + 3; node++)
            {
                if (node <= nodes.Count - 1)
                {
                    if (nodes[node].CompareTag("StopLine"))
                    {
                        approachingStopLine = true;
                    }
                }
            }


            if (nodes[currentNode].CompareTag("StopLine"))
            {
                approachingStopLine = false;
            }

            // Reached the final node in the path
            if (currentNode == nodes.Count - 1)
            {
                // If the path the car is on does not loop it will respawn at its starting point upon reaching the final node in its path
                if (!pathLoops && !stopped)
                {
                    Respawn();
                }
            }

            else if (!deciding)
            {
                currentNode++;
            }
        }
    }

    // Return the car to the carpool
    public void Respawn()
    {
        if (cp == null)
        {
            Destroy(gameObject);
        }
        else
        {
            gameObject.SetActive(false);
            gameObject.transform.SetParent(null);
            cp.ReturnCarToPool(gameObject);
        }
    }

    IEnumerator CheckForNewPath(float time)
    {
        yield return new WaitForSeconds(time);
        // Check for a new path from the player every x seconds
        if (path != null)
        {
            UpdatePath(path);
            maxSpeed = initialMaxSpeed;
        }
    }

    IEnumerator StartDrivingAfterBeingCrashed()
    {
        yield return new WaitForSeconds(5);
        hasCrashed = false;
        targetSpeed = initialMaxSpeed;
    }

    public void UpdatePath(Transform path, PathContainer callbackContainer = null)
    {
        if (path != null)
        {
            this.nodes = path.GetComponentsInChildren<Transform>().Skip(1).ToList();

            this.path = path;

            // set current node to 0, so the car starts at node 0
            currentNode = 0;

            if (callbackContainer != null)
            {
                callbackContainer.SetOccupied(path);
            }

            sensorLength = 0;
        }

    }

    public void UpdateBufferedPath(Transform path)
    {
        this.bufferedPath = path;
    }

    #endregion

    #region Braking/Accelerating
    private void DetermineSpeed(float hitDistance)
    {
        if (!brakeForStopSign)
        {
            if (hitDistance >= minimumDistance)
            {
                targetSpeed = maxSpeed;
            }
            else if (hitDistance <= minimumDistance && hitDistance >= minimumDistance * 0.9f)
            {
                targetSpeed = 0.9f * maxSpeed;
            }
            else if (hitDistance <= minimumDistance * 0.8f && hitDistance >= minimumDistance * 0.7f)
            {
                targetSpeed = 0.8f * maxSpeed;
            }
            else if (hitDistance <= minimumDistance * 0.7f && hitDistance >= minimumDistance * 0.6f)
            {
                targetSpeed = 0.6f * maxSpeed;
            }
            else if (hitDistance <= minimumDistance * 0.6f && hitDistance >= minimumDistance * 0.5f)
            {
                targetSpeed = 0.5f * maxSpeed;
            }
            else if (hitDistance <= minimumDistance * 0.5f && hitDistance >= minimumDistance * 0.3f)
            {
                targetSpeed = 0.3f * maxSpeed;
            }
            else if (hitDistance <= 5)
            {
                targetSpeed = 0;
            }

            if (turning)
            {
                targetSpeed = targetSpeed * 0.5f;
            }
        }
        else
        {
            targetSpeed = 0;
        }
    }

    void LerpBrakeAmount()
    {
        if (targetAccelerationValue > 0)
        {
            currentBrakeValue = 0;
            return;
        }

        if (currentBrakeValue != targetBrakeValue)
        {
            currentBrakeValue = Mathf.Lerp(currentBrakeValue, targetBrakeValue, Time.deltaTime);
        }
        if (targetBrakeValue == 0 && currentBrakeValue < 0.1f)
        {
            currentBrakeValue = 0;
        }
    }

    void LerpAccelerationAmount()
    {
        if (targetBrakeValue > 0)
        {
            currentAccelerationValue = 0;
            return;
        }

        if (currentAccelerationValue != targetAccelerationValue)
        {
            currentAccelerationValue = Mathf.Lerp(currentAccelerationValue, targetAccelerationValue, Time.deltaTime);
        }
        if (targetAccelerationValue == 0 && currentAccelerationValue < 0.1f)
        {
            currentAccelerationValue = 0;
        }
    }

    private void MatchTargetSpeed()
    {
        difference = currentSpeed - targetSpeed;

        // decrease speed
        if (difference > 0)
        {
            targetAccelerationValue = 0;
            ps.Accelerate(0);
            if (difference <= 0)
            {
                // let physics do it's job
            }
            else
            {
                if (difference > 1 && difference <= 3)
                {
                    targetBrakeValue = 0f;
                }
                if (difference > 3 && difference <= 7)
                {
                    targetBrakeValue = 10f;
                }
                if (difference > 7 && difference <= 10)
                {
                    targetBrakeValue = 30f;
                }
                if (difference > 10 && difference <= 15)
                {
                    targetBrakeValue = 50f;
                }
                if (difference > 20 && difference <= 30)
                {
                    targetBrakeValue = 100f;
                }

                // currentBrakeValue is constantly lerping towards targetBrakeValue
                ps.Brake(currentBrakeValue);
            }
        }

        // increase speed
        if (difference < 0)
        {
            targetBrakeValue = 0f;
            ps.Brake(0);

            if (difference >= -3)
            {
                targetAccelerationValue = 5f;
            }
            if (difference <= -3)
            {
                targetAccelerationValue = 100;
            }
            if (!isBrakingForStopSign)
            {
                ps.Accelerate(currentAccelerationValue / 2);
            }
        }
    }
    #endregion

    #region Steering
    // Calculate the angle for the wheelcolliders in order to steer towards the next node
    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currentNode].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        targetSteerAngle = newSteer;
    }

    // Lerp to the angle so the car turns slowly and does not snap to new position
    private void LerpToSteerAngle()
    {
        wheelFL.steerAngle = Mathf.Lerp(wheelFL.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        wheelFR.steerAngle = Mathf.Lerp(wheelFR.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
    }
    #endregion

    #region Other
    private void MeasureWaitingTime()
    {
        // Start and stop counting
        if (currentSpeed == 0)
        {
            timeStandingStill += Time.deltaTime;
        }
        else
        {
            timeStandingStill = 0;
        }
        secondsStandingStill = Mathf.RoundToInt(timeStandingStill);

        // Waiting too long
        if (Mathf.RoundToInt(timeStandingStill) > allowedWaitingTime)
        {
            if (waitingToLong == false)
            {
                waitingToLong = true;
            }

            excessSecFloat += Time.deltaTime;
            excessSecondsStoodStill = Mathf.RoundToInt(excessSecFloat);
        }
        else
        {
            excessSecondsStoodStill = 0;
            excessSecFloat = 0;
            waitingToLong = false;
        }

        if (Mathf.RoundToInt(excessSecondsStoodStill) > allowedWaitingTime * 3)
        {
            if (!hasHonkedForStandingStillTooLong)
            {
                if (GeneralHonkingManager.GHM.HonkingAllowed)
                {
                    StartCoroutine(PlayHonk(1f));
                    StartCoroutine(ShoutCarStoodStillCoroutine(2));
                    hasHonkedForStandingStillTooLong = true;
                    StartCoroutine(GeneralHonkingManager.GHM.DissalowHonkingForSeconds());
                }
            }
        }
    }

    IEnumerator ShoutCarStoodStillCoroutine(int seconds)
    {
        yield return new WaitForSeconds(seconds);
        //CoachFeedback.CF.ShoutFeedback(CoachFeedback.DirectionsToShout.carStoodStillTooLong);
    }

    // Sensors coming from the front of the car to avoid obstacles
    private void Sensors()
    {
        // Every frame set the start position of the sensors to the transform of the vehicle
        RaycastHit rightHit;
        RaycastHit leftHit;
        Vector3 RightSensorStartPos = transform.position;
        RightSensorStartPos += transform.forward * frontSensorPos.z;
        RightSensorStartPos += transform.up * frontSensorPos.y;
        RightSensorStartPos += transform.right * 0.7f;

        Vector3 LeftSensorStartPos = transform.position;
        LeftSensorStartPos += transform.forward * frontSensorPos.z;
        LeftSensorStartPos += transform.up * frontSensorPos.y;
        LeftSensorStartPos -= transform.right * 0.7f;

        Quaternion rightSpread = Quaternion.AngleAxis(0, new Vector3(0, 1, 0));
        Vector3 newRightAngle = rightSpread * RightSensorStartPos;
        Quaternion leftSpread = Quaternion.AngleAxis(0, new Vector3(0, 1, 0));
        Vector3 newLeftAngle = leftSpread * LeftSensorStartPos;

        //// Bit shift the index of the layer (18) to get a bit mask (invisible wall)
        //int layerMask = 1 << 18;

        // This would cast rays only against colliders in layer 18.
        // But instead we want to collide against everything except layer 1s8. The ~ operator does this, it inverts a bitmask.
        //layerMask = ~layerMask;

        // Determine if we need to steer left (negative) or right (positive) to avoid obstacle
        float avoidMultiplier = 0f;
        // Front centre sensor
        if (avoidMultiplier >= -1 /*true*/) //TODO: disabled avoiding
        {
            var futureNodes = nodes.Skip(currentNode);
            var closest = futureNodes.Aggregate((x, y) => Math.Abs(Vector3.Distance(transform.position, x.transform.position) - sensorLength) < Math.Abs(Vector3.Distance(transform.position, y.transform.position) - sensorLength) ? x : y);
            Vector3 nodeWithSetHeight = new Vector3(closest.position.x, RightSensorStartPos.y, closest.position.z);

            float dist = Vector3.Distance(transform.position, closest.position);

            if (dist < 10)
            {
                dist = 80;
            }

            // Right front sensor
            if (Physics.Raycast(RightSensorStartPos, (Quaternion.Euler(0, rightAngleDiff, 0) * nodeWithSetHeight) - transform.position, out rightHit, dist))
            {
                if (!rightHit.collider.CompareTag("Poot"))
                {
                    if (!frontRightSensorSpotted & !frontLeftSensorSpotted)
                    {
                        relevantBrakeDistance = rightHit.distance;
                    }

                    if (rightHit.distance < relevantBrakeDistance)
                    {
                        relevantBrakeDistance = rightHit.distance;
                    }

                    // If it's a car
                    if (rightHit.collider.CompareTag("Car") || rightHit.collider.CompareTag("CarViewArea"))
                    {
                        // We're standing still but the car in front is accelerating 
                        if (currentSpeed < rightHit.transform.GetComponent<trafficAI>().currentSpeed)
                        {
                            DetermineSpeed(rightHit.distance);
                        }

                        // We're braking for a stopsign while the car in front is accelerating so stop braking for stopsign
                        else if(isBrakingForStopSign)
                        {
                            if(!rightHit.transform.GetComponent<trafficAI>().isBrakingForStopSign)
                            {
                                isBrakingForStopSign = false;
                            }
                        }
                        else
                        {
                            DetermineSpeed(relevantBrakeDistance);
                        }
                        frontRightSensorSpotted = true;
                    }
                    // If it's a cyclist or pedestrian
                    if (rightHit.collider.CompareTag("Cyclist") || rightHit.collider.CompareTag("Pedestrian"))
                    {
                        if (rightHit.collider.GetComponent<bikeNoPhysics>().isStoppedAtStopLine) return;
                        else
                        {
                            DetermineSpeed(relevantBrakeDistance);
                        }
                    }
                    // If it's a player
                    if (rightHit.collider.gameObject.layer == 8)
                    {
                        DetermineSpeed(relevantBrakeDistance - 3f);
                        frontRightSensorSpotted = true;
                    }
                    ReasonForBraking = rightHit.transform.gameObject;
                    Debug.DrawLine(RightSensorStartPos, rightHit.point, Color.blue);
                }
            }
            else
            {
                frontRightSensorSpotted = false;
                if (!isBrakingForStopSign && frontLeftSensorSpotted == false)
                {
                    targetSpeed = maxSpeed;
                }
            }

            // Left front sensor
            if (Physics.Raycast(LeftSensorStartPos, (Quaternion.Euler(0, leftAngleDiff, 0) * nodeWithSetHeight) - transform.position, out leftHit, dist))
            {
                if (!leftHit.collider.CompareTag("Poot"))
                {
                    if (!frontRightSensorSpotted & !frontLeftSensorSpotted)
                    {
                        relevantBrakeDistance = leftHit.distance;
                    }

                    if (leftHit.distance < relevantBrakeDistance)
                    {
                        relevantBrakeDistance = leftHit.distance;
                    }
                    // Full-on brake and stop avoiding other things when getting too close to another vehicle 
                    if (leftHit.collider.CompareTag("Car") || leftHit.collider.CompareTag("CarViewArea"))
                    {
                        // We're standing still but the car in front is accelerating 
                        if (currentSpeed < leftHit.transform.GetComponent<trafficAI>().currentSpeed)
                        {
                            DetermineSpeed(leftHit.distance);
                        }
                        // We're braking for a stopsign while the car in front is accelerating so stop braking for stopsign
                        else if (isBrakingForStopSign)
                        {
                            if (!leftHit.transform.GetComponent<trafficAI>().isBrakingForStopSign)
                            {
                                isBrakingForStopSign = false;
                            }
                        }
                        else
                        {
                            DetermineSpeed(relevantBrakeDistance);
                        }

                        frontLeftSensorSpotted = true;
                        ReasonForBraking = leftHit.transform.gameObject;
                        Debug.DrawLine(LeftSensorStartPos, leftHit.point, Color.blue);
                    }
                    if (leftHit.collider.gameObject.layer == 8)
                    {
                        frontLeftSensorSpotted = true;
                        DetermineSpeed(relevantBrakeDistance - 3);
                    }
                }
            }
            // Only reset the acceleration if the other sensor agrees 
            else
            {
                frontLeftSensorSpotted = false;
                if (!isBrakingForStopSign && frontRightSensorSpotted == false)
                {
                    targetSpeed = maxSpeed;
                }
            }

            if (!frontRightSensorSpotted && !frontLeftSensorSpotted)
            {
                relevantBrakeDistance = 0;
            }
        }
    }

    private void ReactToPlayerSigns()
    {
        if (isBrakingForStopSign)
        {
            currentAccelerationValue = 0f;
            if (approachingStopLine)
            {
                if (nodes[currentNode].CompareTag("StopLine"))
                {
                    if (Vector3.Distance(transform.position, nodes[currentNode].position) < 15f)
                    {
                        brakeForStopSign = true;
                    }
                }
            }
            else
            {
                brakeForStopSign = true;
            }
        }
        else
        {
            brakeForStopSign = false;
        }
    }

    private void StopCar()
    {
        maxSpeed = 0;
        isBraking = true;
        stopped = true;
    }

    private void TurnOffRayCast()
    {
        foreach (Transform child in gameObject.GetComponentInChildren<Transform>(true))
        {
            child.gameObject.layer = LayerMask.NameToLayer("Ignore Raycast");
        }
        this.gameObject.layer = LayerMask.NameToLayer("Ignore Raycast");
    }

    IEnumerator PlayHonk(float length)
    {
        // We play it instantly for now. The aggression value should indicate the length of the honk. Not the delay.
        yield return new WaitForSeconds(length);
        //car.PlayOneShot(honkSound);
        AudioSource.PlayClipAtPoint(car.clip, transform.position, 1);
    }

    // The car is touching another car, increment the timer
    public void IncrementCollidingTimer()
    {
        isTouchingCarTimer += Time.deltaTime;
        if (isTouchingCarTimer > 7)
        {
            Respawn();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            playerHit = true;
        }

        if (collision.gameObject.name == "LocalAvatar" || collision.gameObject.CompareTag("Player"))
        {
            foreach (var item in coroutinesToCancel)
            {
                StopCoroutine(item);
            }
            foreach (var item in carWheels)
            {
                item.Rotate(false);
            }
        }


        if (collision.transform.root.CompareTag("Car") && isPositionedOnIntersection)
        {
            isTouchingCar = true;
        }

        // Crashed on the intersection
        if (collision.transform.root.CompareTag("Car") && currentSpeed > 2 && isPositionedOnIntersection && !hasCrashed ||
           collision.transform.root.CompareTag("Car") && collision.transform.GetComponent<trafficAI>().currentSpeed > 2 && isPositionedOnIntersection && !hasCrashed)
        {
            hasCrashed = true;
            if (pm != null)
            {
                pm.amountOfCarsCrashed += 1;
            }
            AudioSource.PlayClipAtPoint(crashSound, transform.position);
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        var collisionName = collision.gameObject.tag;
        if ((collisionName == "LocalAvatar" || collisionName == "Player") && rb.constraints != RigidbodyConstraints.FreezeAll)
        {
            rb.constraints = RigidbodyConstraints.FreezeAll;
            rb.velocity = new Vector3(0, 0, 0);

            foreach (var item in carWheels)
            {
                item.Rotate(false);
            }
        }


    }

    private void OnCollisionExit(Collision collision)
    {
        //Debug.Log($"Exit collision with {collision.gameObject.name}");
        if (collision.gameObject.name == "LocalAvatar" || collision.gameObject.CompareTag("Player"))
        {
            disableConstraintCoroutine = StartCoroutine(SetInitialConstraintsAfterSeconds(0.5f));

            coroutinesToCancel.Add(disableConstraintCoroutine);

            foreach (var wheel in carWheels)
            {
                wheel.Rotate(true);
            }
        }
        if (collision.transform.root.CompareTag("Car"))
        {
            isTouchingCar = false;
        }
    }

    private void CheckIfPositionedOnIntersection()
    {
        if (intersectionCornersPresent)
        {
            if (transform.position.x > NWCorner.x && transform.position.x < SECorner.x && transform.position.z > NWCorner.z && transform.position.z < SECorner.z)
            {
                isPositionedOnIntersection = true;
            }
            else
            {
                isPositionedOnIntersection = false;
            }
        }
    }

    /// <summary>
    /// When a car stands in line it should be considered spotted
    /// </summary>
    public void SetIsSpottedWhenWaitingForTraffic()
    {
        if (currentSpeed == 0 && !speedCheckPerformed)
        {
            StartCoroutine(CheckSpeedAfterSeconds(3));
            speedCheckPerformed = true;
        }
    }

    public IEnumerator CheckSpeedAfterSeconds(int seconds)
    {
        yield return new WaitForSeconds(seconds);
        if (currentSpeed == 0)
        {
            GetComponent<playerInteractions>().isSpotted = true;
        }
        speedCheckPerformed = false;
    }

    /// <summary>
    /// When the car enters the intersection without having been spotted by the player have the coach say something, unless the car was previously stopped because of traffic
    /// </summary>
    public void CoachFeedbackWhenNotSpotted()
    {
        if (isPositionedOnIntersection && !GetComponent<playerInteractions>().isSpotted && !spottedAtIntersectionCoroutineRunning)
        {
            if (CoachFeedback.CF != null)
            {
                CoachFeedback.CF.ShoutFeedback(CoachFeedback.DirectionsToShout.lookAroundMissedCar);
            }
            spottedAtIntersectionCoroutineRunning = true;
        }
    }

    private void CheckBrakeLights()
    {
        if (lights != null)
        {
            if (ps.CurrentBrakeTorque > 15 && currentSpeed != 0)
            {
                lights.SetActive(true);
            }
            else
            {
                lights.SetActive(false);
            }
        }
    }

    private IEnumerator SetInitialConstraintsAfterSeconds(float seconds)
    {
        yield return new WaitForSeconds(seconds);
        rb.constraints = initialConstraints;
    }

    private void ResetWheelColliders()
    {
        // Reset the z rotation to 0 every frame to keep the car from tipping
        transform.rotation = Quaternion.Euler(transform.eulerAngles.x, transform.eulerAngles.y, 0);
    }

    /// <summary>
    /// Failsafe for cars softlocked at the centre of the intersection
    /// </summary>
    private void MidIntersectionFailsafe()
    {
        if (secondsStandingStill > 15 && isPositionedOnIntersection && !isBrakingForStopSign)
        {
            Respawn();
        }
    }
    #endregion
}
