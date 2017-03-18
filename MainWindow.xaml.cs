//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Linq;
    using System;
    using System.Drawing;
    using System.Collections.Generic;
    using Emgu.CV;
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// The instance of bluetoothService used for all Bluetooth calls
        /// </summary>
        private BluetoothService bluetoothService;

        /// <summary>
        /// The renderer tasked with rendering the skeleton for display purposes.
        /// Code mostly taken from the sample
        /// </summary>
        private SkeletonRenderer renderer;

        /// <summary>
        /// Finds the marker locations given the colour image data
        /// </summary>
        private MarkerFinder finder;

        /// <summary>
        /// The class for communicating with the Atmel board
        /// </summary>
        private SerialComms comms;

        /// <summary>
        /// The class which will run our game
        /// </summary>
        private GameLogic gameLogic;

        /// <summary>
        /// The time of the last skeleton sending
        /// </summary>
        private DateTime lastSkeletonSent = new DateTime(0);

        /// <summary>
        /// The time of the last skeleton sending
        /// </summary>
        private DateTime lastPosSent = new DateTime(0);

        /// <summary>
        /// The last head position sent
        /// </summary>
        private List<double> lastHeadPosSent = null;

        /// <summary>
        /// Which joints we will send over bluetooth. All others ignored
        /// </summary>
        private static readonly JointType[] JointsToSend = { JointType.Head, JointType.HandLeft,
            JointType.HandRight, JointType.WristLeft, JointType.WristRight };

        /// <summary>
        /// DrawingGroup used to draw euro filter results
        /// </summary>
        private DrawingGroup euroDrawingGroup;

        /// <summary>
        /// Brush used to draw euro filtered points
        /// </summary>
        private readonly System.Windows.Media.Brush euroOutput = System.Windows.Media.Brushes.Magenta;

        /// <summary>
        /// The last calibrated robot position
        /// </summary>
        Matrix<double> calibRobPos = null;

        /// <summary>
        /// The last calibrated robot rotation matrix
        /// </summary>
        Matrix<double> calibRobRotMatrix = null;

        /// <summary>
        /// The vertical offset from the center of the robot to the ground (0.225)
        /// and the distance from the ball joints to the interaction plate (0.034)
        /// </summary>
        double robotOriginVerticalOffset = 0.259;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Start bluetooth service
            bluetoothService = new BluetoothService();
            bluetoothService.StartBluetooth();

            // Create the drawing group we'll use for drawing
            DrawingGroup drawingGroup = new DrawingGroup();
            euroDrawingGroup = new DrawingGroup();
            DrawingGroup markerDrawingGroup = new DrawingGroup();

            // Create image sources that we can use in our image controls
            ImageSource imageSource = new DrawingImage(drawingGroup);
            ImageSource euroSource = new DrawingImage(euroDrawingGroup);
            ImageSource markerSource = new DrawingImage(markerDrawingGroup);

            // Display the drawing using our image controls
            Image.Source = imageSource;
            EuroOverlay.Source = euroSource;
            MarkerOverlay.Source = markerSource;

            // Start up comms
            comms = new SerialComms(EnableButton);
            comms.kinPosUpdated += this.kinPositionUpdated;

            // Start our game
            gameLogic = new GameLogic(comms, bluetoothService, this.GameButton);

            // Start up the marker finder
            this.finder = new MarkerFinder(markerDrawingGroup);


            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.renderer = new SkeletonRenderer(drawingGroup, this.sensor.CoordinateMapper);
                this.sensor.SkeletonStream.Enable();
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Enable the camera to track the markers
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution1280x960Fps12);
                this.sensor.ColorFrameReady += this.ColorFrameReady;

                // Enable the depth camera to find the depth of each marker
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                this.sensor.DepthFrameReady += this.DepthFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// A method to handle a new depth frame. Saves the mapping
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame frame = e.OpenDepthImageFrame())
            {
                if (frame == null) return;
                DepthImagePixel[] pixelData = new DepthImagePixel[frame.Width * frame.Height];
                frame.CopyDepthImagePixelDataTo(pixelData);
                //this.sensor.CoordinateMapper.MapColorFrameToSkeletonFrame(sensor.ColorStream.Format, DepthImageFormat.Resolution640x480Fps30, pixelData, lastSkeletonMapping);
                //skeletonMapped = true;
            }
        }

        /// <summary>
        /// A method to handle a new color frame. Extracts marker locations
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            List<int> idList = new List<int>();
            List<double[]> rotationList = new List<double[]>();
            List<double[]> translationList = new List<double[]>();
            List<double> headPos = new List<double>();
            List<double> headRotMatrix = new List<double>();
            List<double> robPos = new List<double>();
            List<double> robRotMatrix = new List<double>();
            double deltaT = 0;
            System.Threading.ThreadPool.QueueUserWorkItem(delegate {
                int markerCount = finder.FindMarkers(e, idList, rotationList, translationList, ref deltaT, headPos, headRotMatrix, robPos, robRotMatrix);

                if (markerCount == 0 || (headPos.Count == 0 && robPos.Count == 0)) return;

                string stringToSend = "MLoc," + (int)deltaT + ",";

                if (headPos.Count != 0)
                {
                    if (calibRobPos != null)
                    {
                        headPos[0] = -headPos[0];
                        headPos[1] = calibRobPos[1, 0] - headPos[1];
                        stringToSend = stringToSend + "HED," + String.Join(",", headPos) + ",";
                        if (headRotMatrix.Count > 0)
                        {
                            stringToSend = stringToSend + String.Join(",", headRotMatrix) + ",";
                        }
                        lastHeadPosSent = headPos;
                    }
                    else  if (robPos.Count == 0) return;
                }
                if (robPos.Count != 0)
                {
                    robPos[0] = -robPos[0];
                    robPos[1] = robPos[1];
                    List<double> robRotMatrixTransform = new List<double>();
                    for (int i = 0; i < robRotMatrix.Count; i++)
                    {
                        if (i < 6) { // First two rows must be negative
                            robRotMatrixTransform.Add(-robRotMatrix[i]);
                        } else {
                            robRotMatrixTransform.Add(robRotMatrix[i]);
                        }
                    }
                    stringToSend = stringToSend + "ROB," + String.Join(",", robPos) + "," + String.Join(",", robRotMatrixTransform) + ",";
                    calibRobPos = new Matrix<double>(robPos.ToArray());
                    calibRobRotMatrix = new Matrix<double>(3,3);
                    for(int i = 0; i < 9; i++)
                    {
                        calibRobRotMatrix.Data[i / 3, i % 3] = robRotMatrixTransform[i];
                    }
                }
                
                bluetoothService.Send(stringToSend);
            }, null);
        }

        /// <summary>
        /// Normalizes a skeleton point into a unit vector
        /// </summary>
        /// <param name="point">The point to normalize</param>
        public void normalizeVector(SkeletonPoint point)
        {
            float pointLength = (float)Math.Sqrt(Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2));
            point.X = point.X / pointLength;
            point.Y = point.Y / pointLength;
            point.Z = point.Z / pointLength;
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            Skeleton skeletonToSend = null;
            List<Skeleton> trackedSkeletons = skeletons.Where(skeleton => skeleton.TrackingState == SkeletonTrackingState.Tracked).ToList();
            if(lastHeadPosSent != null)
            {
                foreach(Skeleton skeleton in trackedSkeletons)
                {
                    if(skeletonToSend == null)
                    {
                        skeletonToSend = skeleton;
                        continue;
                    }
                    SkeletonPoint sendHead = skeletonToSend.Joints.Where(joint => joint.JointType == JointType.Head).FirstOrDefault().Position;
                    SkeletonPoint candHead = skeleton.Joints.Where(joint => joint.JointType == JointType.Head).FirstOrDefault().Position;
                    double sendDistSquared = Math.Pow(sendHead.X - lastHeadPosSent[0], 2) + Math.Pow(sendHead.Y - lastHeadPosSent[1], 2) +
                        Math.Pow(sendHead.Z - lastHeadPosSent[2], 2);
                    double candDistSquared = Math.Pow(candHead.X - lastHeadPosSent[0], 2) + Math.Pow(candHead.Y - lastHeadPosSent[1], 2) +
                        Math.Pow(candHead.Z - lastHeadPosSent[2], 2);

                    if (candDistSquared < sendDistSquared) skeletonToSend = skeleton;
                }
            }

            if (skeletonToSend != null && calibRobPos != null)
            {
                string stringToSend = "JLoc,";

                DateTime tempNow = DateTime.Now;
                int deltaT = (int)tempNow.Subtract(lastSkeletonSent).TotalMilliseconds;
                stringToSend = stringToSend + deltaT + ",";
                lastSkeletonSent = tempNow;
                double rate = 1000 / (double)deltaT;
                if (deltaT == 0)
                {
                    rate = double.MaxValue;
                }
                foreach (Joint joint in skeletonToSend.Joints)
                {
                    double jointX = joint.Position.X - 0.025;
                    double jointY = joint.Position.Y + calibRobPos[1,0];
                    double jointZ = joint.Position.Z;

                    if (!(bool) DisplayCheckBox.IsChecked && (JointType.HandLeft == joint.JointType || JointType.HandRight == joint.JointType)) jointY = -15;

                    int filtersIndex = Array.IndexOf(JointsToSend, joint.JointType) * 3;
                    if (filtersIndex < 0) continue;

                    stringToSend = stringToSend + "JNT," +
                        (int)joint.JointType + "," +
                        (int)joint.TrackingState + "," +
                        jointX + "," + jointY + "," + jointZ + ",";
                }
                if (stringToSend.Contains("NaN"))
                {
                    return;
                }
                bluetoothService.Send(stringToSend);
            }

            this.renderer.RenderSkeletons(skeletons, skeletonToSend);
        }

        /// <summary>
        /// Handle a position update from the forward kinematics
        /// </summary>
        /// <param name="position">The position of the robot</param>
        /// <param name="force">Ignored</param>
        private void kinPositionUpdated(double[] position, double force)
        {
            if (calibRobPos == null) return;

            string stringToSend = "RPos,";

            if (!gameLogic.isInteractable)
            {
                if (lastPosSent.Ticks > 0) return;

                // Send sentinel value because the end effector is no longer interactable
                lastPosSent = new DateTime(0);
                stringToSend = stringToSend + "0,0,0,0,";
                bluetoothService.Send(stringToSend);
                return;
            }

            DateTime tempNow = DateTime.Now;
            stringToSend = stringToSend + (int)tempNow.Subtract(lastPosSent).TotalMilliseconds + ",";
            lastPosSent = tempNow;

            //Matrix<double> positionMat = new Matrix<double>(position).Transpose();
            Matrix<double> positionMat = new Matrix<double>(3,1);
            positionMat.Data[0, 0] = -position[1];
            positionMat.Data[1, 0] = position[2] + robotOriginVerticalOffset;
            positionMat.Data[2, 0] = -position[0];
            positionMat = calibRobPos + calibRobRotMatrix * positionMat;

            double xToSend = positionMat.Data[0, 0];
            double yToSend = positionMat.Data[1, 0] - calibRobPos[1, 0];
            double zToSend = positionMat.Data[2, 0];
            stringToSend = stringToSend + xToSend + "," + yToSend + "," + zToSend + ",";
            Console.WriteLine(stringToSend);
            bluetoothService.Send(stringToSend);
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
            if (null != this.comms)
            {
                this.comms.Stop();
            }
            if (null != this.gameLogic)
            {
                this.gameLogic.Stop();
            }
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        /// <summary>
        /// Handle a KeyDown event in the textbox, filtering for returns
        /// </summary>
        /// <param name="sender">Unused</param>
        /// <param name="e">Unused</param>
        private void SerialCommandBox_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == System.Windows.Input.Key.Return)
            {
                if (String.IsNullOrEmpty(SerialCommandBox.Text)) return;
                comms.HandleCommand(SerialCommandBox.Text);
                SerialCommandBox.Text = "";
            }
        }

        /// <summary>
        /// Handle a click on the enable button
        /// </summary>
        /// <param name="sender">Unused</param>
        /// <param name="e">Unused</param>
        private void EnableButton_Click(object sender, RoutedEventArgs e)
        {
            comms.HandleCommand((string)EnableButton.Content);
        }
    }
}