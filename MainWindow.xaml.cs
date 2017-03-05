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
    using System.Collections.Generic;    /// <summary>
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
        /// The time of the last skeleton sending
        /// </summary>
        DateTime lastSkeletonSent = new DateTime(0);

        /// <summary>
        /// The time of the last skeleton sending
        /// </summary>
        DateTime lastPosSent = new DateTime(0);

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
            DrawingGroup markerDrawingGroup = new DrawingGroup();

            // Create image sources that we can use in our image controls
            ImageSource imageSource = new DrawingImage(drawingGroup);
            ImageSource markerSource = new DrawingImage(markerDrawingGroup);

            // Display the drawing using our image controls
            Image.Source = imageSource;
            MarkerOverlay.Source = markerSource;

            // Start up comms
            comms = new SerialComms(this.positionUpdated, EnableButton);

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
                this.renderer = new SkeletonRenderer(drawingGroup, this.sensor.CoordinateMapper);
                this.finder = new MarkerFinder(markerDrawingGroup);

                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
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
            double deltaT = 0, goggleAngle = 0;
            int markerCount = finder.FindMarkers(e, idList, rotationList, translationList, ref deltaT, ref goggleAngle, headPos);

            if (markerCount == 0) return;

            string stringToSend = "MLoc," + (int)deltaT + ",";

            stringToSend = stringToSend + "HED," + headPos[0] + "," + headPos[1] + "," + headPos[2] + "," + goggleAngle + ",";
            
            bluetoothService.Send(stringToSend);
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
            var skeletonToSend = skeletons.Where(skeleton => skeleton.TrackingState == SkeletonTrackingState.Tracked).Select(skeleton => skeleton).FirstOrDefault();
            if (skeletonToSend != null)
            {
                string stringToSend = "JLoc,";

                DateTime tempNow = DateTime.Now;
                stringToSend = stringToSend + (int)tempNow.Subtract(lastSkeletonSent).TotalMilliseconds + ",";
                lastSkeletonSent = tempNow;

                // Order: SC, HD, WL, HL, WR, HR
                foreach (Joint joint in skeletonToSend.Joints)
                {
                    stringToSend = stringToSend + "JNT," +
                        (int)joint.JointType + "," +
                        (int)joint.TrackingState + "," +
                        joint.Position.X + "," +
                        joint.Position.Y + "," +
                        joint.Position.Z + ",";
                }
                bluetoothService.Send(stringToSend);
            }

            this.renderer.RenderSkeletons(skeletons);
        }

        /// <summary>
        /// Handle a position update from the forward kinematics
        /// </summary>
        /// <param name="position">The position of the robot</param>
        private void positionUpdated(double[] position)
        {
            string stringToSend = "Rpos,";

            DateTime tempNow = DateTime.Now;
            stringToSend = stringToSend + (int)tempNow.Subtract(lastPosSent).TotalMilliseconds + ",";
            lastPosSent = tempNow;

            stringToSend = stringToSend + position[0] + position[1] + position[2];
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