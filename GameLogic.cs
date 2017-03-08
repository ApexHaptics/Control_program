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
        private const float work_radius = 0.2f;

        /// <summary>
        /// The lower z of our workspace
        /// </summary>
        private const float work_z_low = 0.5f;

        /// <summary>
        /// The upper z of our workspace
        /// </summary>
        private const float work_z_high = 1f;

        /// <summary>
        /// The distance before we consider the target "reached" (currently 0.05^2)
        /// </summary>
        private const double dist_thresh_square = 0.0025;

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
        /// A queue of arrived positions to compare with the demanded position
        /// </summary>
        BlockingCollection<double[]> positionQueue = new BlockingCollection<double[]>(new ConcurrentQueue<double[]>());

        /// <summary>
        /// The last position the robot was told to move to
        /// </summary>
        private double[] lastCommandedPosition;

        /// <summary>
        /// Whether threads should continue
        /// </summary>
        bool _continue = true;

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
            this.btService = btService;
            gameThread = new Thread(this.GameLoop);
            gameButton.Click += GameButton_Click;
        }

        /// <summary>
        /// Starts the game loop
        /// </summary>
        private void GameButton_Click(object sender, System.Windows.RoutedEventArgs e)
        {
            this.gameThread.Start();
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
            float x, y, z;
            while (_continue)
            {
                MakeRandomCirclePoints(out x, out y);
                z = work_z_low;
                lastCommandedPosition = new double[] { x, y, z };
                comms.SetTargetPosition(x, y, z);

                while (true) // Block until we arrive at the end
                {
                    double[] pos = positionQueue.Take();
                    double dist_square = Math.Pow(lastCommandedPosition[0] - pos[0], 2) +
                        Math.Pow(lastCommandedPosition[1] - pos[1], 2) +
                        Math.Pow(lastCommandedPosition[2] - pos[2], 2);
                    if (dist_square < dist_thresh_square) break;
                }
                // We reached it: inform the display
            }
        }

        /// <summary>
        /// Stops the game thread
        /// </summary>
        public void Stop()
        {
            _continue = false;
            positionQueue.Add(lastCommandedPosition);
            if (gameThread != null)
            {
                gameThread.Join();
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
