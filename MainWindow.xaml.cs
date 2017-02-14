﻿//------------------------------------------------------------------------------
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
        /// The last recorded mapping from colour space to skeleton space
        /// </summary>
        private SkeletonPoint[] lastSkeletonMapping = new SkeletonPoint[640 * 480];

        /// <summary>
        /// Whether <see cref="lastSkeletonMapping"/> is initialized
        /// </summary>
        private bool skeletonMapped = false;

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
                this.finder = new MarkerFinder(markerDrawingGroup);

                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.renderer.SensorSkeletonFrameReady;

                // Enable the camera to track the markers
                this.sensor.ColorStream.Enable();
                this.sensor.ColorFrameReady += this.ColorFrameReady;

                // Enable the depth camera to find the depth of each marker
                this.sensor.DepthStream.Enable();
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
                this.sensor.CoordinateMapper.MapColorFrameToSkeletonFrame(sensor.ColorStream.Format, DepthImageFormat.Resolution640x480Fps30, pixelData, lastSkeletonMapping);
                skeletonMapped = true;
            }
        }

        /// <summary>
        /// A method to handle a new color frame. Extracts marker locations
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            PointF[][] markerArray = finder.FindMarkers(e);

            if (markerArray == null || markerArray.Length == 0 || skeletonMapped == false) return;

            string stringToSend = "MLoc,";
            
            // Important to note is that marker vertices are counterclockwise starting from top left
            foreach (PointF[] marker in markerArray)
            {
                // Find center before converting to skeleton space. Lag between color
                // frames and depth frames could cause problems as seen in Kinect Studio
                // Using https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
                float den = (marker[0].X - marker[2].X) * (marker[1].Y - marker[3].Y) -
                    (marker[0].Y - marker[2].Y) * (marker[1].X - marker[3].X);
                float xNum = (marker[0].X * marker[2].Y - marker[0].Y * marker[2].X) * (marker[1].X - marker[3].X) -
                    (marker[1].X * marker[3].Y - marker[1].Y * marker[3].X) * (marker[0].X - marker[2].X);
                float yNum = (marker[0].X * marker[2].Y - marker[0].Y * marker[2].X) * (marker[1].Y - marker[3].Y) -
                    (marker[1].X * marker[3].Y - marker[1].Y * marker[3].X) * (marker[0].Y - marker[2].Y);

                SkeletonPoint centerPoint = lastSkeletonMapping[(int)(yNum / den * sensor.ColorStream.FrameWidth + xNum / den)];
                SkeletonPoint point0 = lastSkeletonMapping[(int)(marker[0].Y * sensor.ColorStream.FrameWidth + marker[0].X)];
                SkeletonPoint point1 = lastSkeletonMapping[(int)(marker[1].Y * sensor.ColorStream.FrameWidth + marker[1].X)];
                SkeletonPoint point2 = lastSkeletonMapping[(int)(marker[2].Y * sensor.ColorStream.FrameWidth + marker[2].X)];
                SkeletonPoint point3 = lastSkeletonMapping[(int)(marker[3].Y * sensor.ColorStream.FrameWidth + marker[3].X)];

                // If the center point is at 0, the depth sensor couldn't map it and nothing can be used
                if (centerPoint.Z == 0) continue;
                // TODO: Each marker should have an ID when sent
                stringToSend = stringToSend + "MKR:" + centerPoint.X + "," + centerPoint.Y + "," + centerPoint.Z + ",";

                // If any other points are at 0, we can't calculate the normal
                if (point0.Z == 0 || point1.Z == 0 || point2.Z == 0 || point3.Z == 0) return;


                // Calculate line vectors between points 0 and 2 and 1 and 3 to do normal
                SkeletonPoint u = new SkeletonPoint();
                SkeletonPoint v = new SkeletonPoint();
                u.X = point2.X - point0.X;
                u.Y = point2.Y - point0.Y;
                u.Z = point2.Z - point0.Z;
                v.X = point3.X - point1.X;
                v.Y = point3.Y - point1.Y;
                v.Z = point3.Z - point1.Z;

                SkeletonPoint normal = new SkeletonPoint();
                normal.X = (u.Y * v.Z) - (u.Z * v.Y);
                normal.Y = (u.Z * v.X) - (u.X * v.Z);
                normal.Z = (u.X * v.Y) - (u.Y * v.X);

                stringToSend = stringToSend + "NML:" + normal.X + "," + normal.Y + "," + normal.Z + ",";
            }
            System.Diagnostics.Debug.Write("Send:" + stringToSend + "\n");
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
                // Send the hand joint information over bluetooth. This protocol needs to be reworked
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