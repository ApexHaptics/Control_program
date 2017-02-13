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
                this.finder = new MarkerFinder(markerDrawingGroup, this.sensor.CoordinateMapper);

                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.renderer.SensorSkeletonFrameReady;

                // Enable the camera to track the markers
                this.sensor.ColorStream.Enable();
                this.sensor.ColorFrameReady += this.ColorFrameReady;

                this.sensor.DepthStream.Enable();

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
        /// A method to handle a new color frame. Extracts marker locations
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            finder.FindMarkers(e);
            // Method should eventually return data to send
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
                // Send the hand joint information over bluetooth. This protocol nees to be reworked
                // Also these are in skeleton space. They need to be in depth space
                bluetoothService.Send("HD_X," + skeletonToSend.Joints[JointType.Head].Position.X + "," +
                                        "HD_Y," + skeletonToSend.Joints[JointType.Head].Position.Y + "," +
                                        "HD_Z," + skeletonToSend.Joints[JointType.Head].Position.Z + "," +
                                        "SC_X," + skeletonToSend.Joints[JointType.ShoulderCenter].Position.X + "," +
                                        "SC_Y," + skeletonToSend.Joints[JointType.ShoulderCenter].Position.Y + "," +
                                        "SC_Z," + skeletonToSend.Joints[JointType.ShoulderCenter].Position.Z + "," +
                                        "WR_X," + skeletonToSend.Joints[JointType.WristRight].Position.X + "," +
                                        "WR_Y," + skeletonToSend.Joints[JointType.WristRight].Position.Y + "," +
                                        "WR_Z," + skeletonToSend.Joints[JointType.WristRight].Position.Z + "," +
                                        "HR_X," + skeletonToSend.Joints[JointType.HandRight].Position.X + "," +
                                        "HR_Y," + skeletonToSend.Joints[JointType.HandRight].Position.Y + "," +
                                        "HR_Z," + skeletonToSend.Joints[JointType.HandRight].Position.Z + "," +
                                        "WL_X," + skeletonToSend.Joints[JointType.WristLeft].Position.X + "," +
                                        "WL_Y," + skeletonToSend.Joints[JointType.WristLeft].Position.Y + "," +
                                        "WL_Z," + skeletonToSend.Joints[JointType.WristLeft].Position.Z + "," +
                                        "HL_X," + skeletonToSend.Joints[JointType.HandLeft].Position.X + "," +
                                        "HL_Y," + skeletonToSend.Joints[JointType.HandLeft].Position.Y + "," +
                                        "HL_Z," + skeletonToSend.Joints[JointType.HandLeft].Position.Z + ",");
            }

            this.renderer.SensorSkeletonFrameReady(sender, e);
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
    }
}