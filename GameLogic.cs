using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Controls;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class GameLogic
    {
        /// <summary>
        /// The radius of our workspace
        /// </summary>
        private const float work_radius = 0.25f;

        /// <summary>
        /// The lower z of our workspace
        /// </summary>
        private const float work_z_low = 0.79f;

        /// <summary>
        /// The interaction z of our workspace, halfway between 1.04 and 0.79
        /// </summary>
        private const float work_z_mid = 0.92f;

        /// <summary>
        /// The distance before we consider the target "reached" (currently 0.05^2)
        /// </summary>
        private const double dist_thresh_square = 0.0025;

        /// <summary>
        /// The epsilon below which the velocity is considered settled (0.01 m/s)^2
        /// </summary>
        private const double epsilon_square = 0.0001;

        /// <summary>
        /// How long to delay after certain end effector movements
        /// </summary>
        private int movementDelay = 500;

        /// <summary>
        /// The PRNG for this class
        /// </summary>
        private Random rand = new Random();

        /// <summary>
        /// The comms over which we will instruct the robot to move
        /// </summary>
        private SerialComms comms;

        /// <summary>
        /// The service to send game packets to the display
        /// </summary>
        private BluetoothService btService;

        /// <summary>
        /// The thread on which the game logic runs
        /// </summary>
        private Thread gameThread;

        /// <summary>
        /// The button which will be used to start the game and skip interaction phases
        /// </summary>
        private Button gameButton;

        /// <summary>
        /// If true we should skip the current interaction phase
        /// </summary>
        private bool skipInteraction = false;

        /// <summary>
        /// A queue of arrived positions to compare with the demanded position
        /// </summary>
        BlockingCollection<double[]> positionQueue = new BlockingCollection<double[]>(new ConcurrentQueue<double[]>());

        /// <summary>
        /// The last position the robot was told to move to
        /// </summary>
        private double[] lastCommandedPosition;

        /// <summary>
        /// The last position the robot was recorded at
        /// </summary>
        private double[] lastRecordedPosition;

        /// <summary>
        /// The time of lastRecordedPosition
        /// </summary>
        DateTime lastPositionTime;

        /// <summary>
        /// Impedance values to be used by the robot {m,b,k}
        /// </summary>
        float[][] impedanceValues = new float[][]
        {
            new float[]{10, 60, 0},
            new float[]{0, 50, 500},
            new float[]{10, 40, 150},
        };

        /// <summary>
        /// The current impendence selected
        /// </summary>
        int currentImpendenceIndex = 0;

        /// <summary>
        /// Whether threads should continue
        /// </summary>
        private bool _continue = true;

        /// <summary>
        /// Whether the end effector should be interacted with
        /// </summary>
        public bool isInteractable = false;

        /// <summary>
        /// Constructor for the GameLogic class
        /// </summary>
        /// <param name="comms">The application's com communications instance</param>
        /// <param name="btService">The application's bluetooth instance</param>
        /// <param name="gameButton">The button which controls game functions</param>
        public GameLogic(SerialComms comms, BluetoothService btService, Button gameButton)
        {
            this.comms = comms;
            comms.kinPosUpdated += Comms_kinPosUpdated;
            comms.enabledStateUpdated += Comms_enabledStateUpdated;
            this.btService = btService;
            this.gameThread = new Thread(this.GameLoop);
            this.gameButton = gameButton;
            gameButton.Click += GameButton_Click;
        }
        
        /// <summary>
        /// If looping stops the current game loop and prepares for another if the robot is disabled
        /// </summary>
        /// <param name="isEnabled"></param>
        private void Comms_enabledStateUpdated(bool isEnabled)
        {
            System.Windows.Media.Brush buttonColor;

            ThreadPool.QueueUserWorkItem(delegate {
                if (isEnabled)
                {
                    buttonColor = System.Windows.Media.Brushes.DarkGreen;
                }
                else
                {
                    Stop();
                    this.gameThread = new Thread(this.GameLoop);
                    buttonColor = System.Windows.Media.Brushes.DarkGray;
                }
                
                gameButton.Dispatcher.BeginInvoke(new Action(() => {
                    gameButton.Content = "Start Game";
                    gameButton.Background = buttonColor;
                    gameButton.IsEnabled = isEnabled;
                }));
            }, null);
        }

        /// <summary>
        /// Starts the game loop
        /// </summary>
        private void GameButton_Click(object sender, System.Windows.RoutedEventArgs e)
        {
            _continue = true;
            skipInteraction = false;
            if (!gameThread.IsAlive)
            {
                this.gameThread.Start();
            }
            else
            {
                skipInteraction = true;
            }
            gameButton.Content = "In progress";
            gameButton.IsEnabled = false;
            gameButton.Background = System.Windows.Media.Brushes.DarkGray;
        }

        /// <summary>
        /// Handler for new kinematic position data
        /// </summary>
        /// <param name="position">The end effector position</param>
        private void Comms_kinPosUpdated(double[] position)
        {
            positionQueue.Add(position);
        }

        /// <summary>
        /// The loop in which all game logic will occur
        /// </summary>
        private void GameLoop()
        {
            lastRecordedPosition = positionQueue.Take();
            if (lastRecordedPosition == null) return;

            lastPositionTime = DateTime.Now;
            float x = (float)lastRecordedPosition[0],
                y = (float)lastRecordedPosition[1],
                z = (float)lastRecordedPosition[2];

            while (_continue)
            {
                // Set impedance to special uninteractable value while moving
                comms.SetImpedance(0, 0, 0);

                // Step 1: Go down to low z
                z = work_z_low;
                MoveToPoint(x, y, z);
                if (!_continue) return;
                Thread.Sleep(movementDelay);

                // Step 2: Random x, y and still low z
                MakeRandomCirclePoints(out x, out y);
                MoveToPoint(x, y, z);
                if (!_continue) return;
                Thread.Sleep(movementDelay);

                // Step 3: Set impedance
                comms.SetImpedance(impedanceValues[currentImpendenceIndex][0],
                    impedanceValues[currentImpendenceIndex][1],
                    impedanceValues[currentImpendenceIndex][2]);
                InformDisplayOfGameState(currentImpendenceIndex);
                currentImpendenceIndex = (currentImpendenceIndex + 1) % impedanceValues.Length;
                this.isInteractable = true;

                // Step 4: Interaction z, previous x,y
                z = work_z_mid;
                MoveToPoint(x, y, z);
                if (!_continue) return;

                // Step 5: Wait 10s or for button pressed
                gameButton.Dispatcher.BeginInvoke(new Action(() => {
                    gameButton.Content = "Skip";
                    gameButton.IsEnabled = true;
                    gameButton.Background = System.Windows.Media.Brushes.DarkRed;
                }));
                for (int i = 0; i < 20; i++)
                {
                    if (skipInteraction) break;
                    Thread.Sleep(500);
                }

                // Wait over. Prepare for next loop
                skipInteraction = false;
                gameButton.Dispatcher.BeginInvoke(new Action(() => {
                    gameButton.Content = "In progress";
                    gameButton.IsEnabled = false;
                    gameButton.Background = System.Windows.Media.Brushes.DarkGray;
                }));
                this.isInteractable = false;
            }
        }

        /// <summary>
        /// Informs the display of the current stage in game logic
        /// </summary>
        /// <param name="data">An integer which contains game state data</param>
        private void InformDisplayOfGameState(int data = 0)
        {
            string stringToSend = "GStt," + data;
            btService.Send(stringToSend);
        }

        /// <summary>
        /// Move the end effector to a point and blocks until it arrives
        /// </summary>
        /// <param name="x">X pos</param>
        /// <param name="y">Y pos</param>
        /// <param name="z">Z pos</param>
        private void MoveToPoint(float x, float y, float z)
        {
            lastCommandedPosition = new double[] { x, y, z };
            comms.SetTargetPosition(x, y, z);

            while (true)
            {
                double[] pos = positionQueue.Take();
                if (pos == null) return;
                double dist_square = Math.Pow(lastCommandedPosition[0] - pos[0], 2) +
                    Math.Pow(lastCommandedPosition[1] - pos[1], 2) +
                    Math.Pow(lastCommandedPosition[2] - pos[2], 2);
                DateTime now = DateTime.Now;
                double instant_velocity = Math.Pow(lastRecordedPosition[0] - pos[0], 2) +
                    Math.Pow(lastRecordedPosition[1] - pos[1], 2) +
                    Math.Pow(lastRecordedPosition[2] - pos[2], 2) /
                    Math.Pow((lastPositionTime - now).Milliseconds, 2) * 1000000;
                lastRecordedPosition = pos;
                lastPositionTime = now;
                if (dist_square < dist_thresh_square && instant_velocity < epsilon_square) break;
            }
        }

        /// <summary>
        /// Stops the game thread
        /// </summary>
        public void Stop()
        {
            _continue = false;
            skipInteraction = true;
            this.isInteractable = false;
            if (gameThread != null && gameThread.IsAlive)
            {
                positionQueue.Add(null);
                gameThread.Join();
                positionQueue = new BlockingCollection<double[]>(new ConcurrentQueue<double[]>());
            }
        }

        /// <summary>
        /// Generates random points in the robot's circle
        /// </summary>
        /// <param name="x">The x-value of the point</param>
        /// <param name="y">The y-value of the point</param>
        private void MakeRandomCirclePoints(out float x, out float y)
        {
            double sqrt_r = Math.Sqrt(rand.NextDouble())*work_radius;
            double angle = rand.NextDouble() * 2 * Math.PI;
            x = (float)(sqrt_r * Math.Cos(angle));
            y = (float)(sqrt_r * Math.Sin(angle));
        }
    }
}
