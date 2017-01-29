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
    using InTheHand.Net;
    using InTheHand.Net.Sockets;
    using InTheHand.Windows.Forms;
    using System.Net.Sockets;
    using InTheHand.Net.Bluetooth;
    using System.Net;
    using System;
    using System.Diagnostics;
    using System.Threading;
    using System.Windows.Forms;
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Winform Sample Declarations
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        #endregion

        /// <summary>
        /// The Guid of our bluetooth service
        /// </summary>
        readonly Guid OurServiceClassId = new Guid("{2611ba68-84e1-4842-a15e-0bfc7e096686}");

        // Code taken from 32Feet.net "Chat2Device" sample

        /// <summary>
        /// Our service name for bluetooth
        /// </summary>
        readonly string OurServiceName = "Apex PC";
        //
        volatile bool _closing;
        TextWriter _connWtr;
        BluetoothListener _lsnr;

        //--------

        // We need one connection to a remote application.  The bidirectional
        // chat messages are sent and received on that one connection, each on
        // a new-line (terminated with CR+LF).
        // We start a listener to accept incoming connections.  We have a
        // menu-option to connect to a remote device.  If another connection
        // is open then we will disallow a user's attempt to connect outwards
        // and will discard any incoming connections.

        #region Bluetooth start/Connect/Listen
        private void StartBluetooth()
        {
            try
            {
                new BluetoothClient();
            }
            catch (Exception ex)
            {
                var msg = "Bluetooth init failed: " + ex;
                System.Windows.MessageBox.Show(msg);
                throw new InvalidOperationException(msg, ex);
            }
            // TODO Check radio?
            //
            // Always run server?
            StartListener();
        }

        BluetoothAddress BluetoothSelect()
        {
            var dlg = new SelectBluetoothDeviceDialog();
            var rslt = dlg.ShowDialog();
            //if (rslt != DialogResult.OK) TODO: Figure this out
            //{
            //    AddMessage(MessageSource.Info, "Cancelled select device.");
            //    return null;
            //}
            var addr = dlg.SelectedDevice.DeviceAddress;
            return addr;
        }

        void BluetoothConnect(BluetoothAddress addr)
        {
            var cli = new BluetoothClient();
            try
            {
                cli.Connect(addr, OurServiceClassId);
                var peer = cli.GetStream();
                SetConnection(peer, true, cli.RemoteEndPoint);
                ThreadPool.QueueUserWorkItem(ReadMessagesToEnd_Runner, peer);
            }
            catch (SocketException ex)
            {
                // Try to give a explanation reason by checking what error-code.
                // http://32feet.codeplex.com/wikipage?title=Errors
                // Note the error codes used on MSFT+WM are not the same as on
                // MSFT+Win32 so don't expect much there, we try to use the
                // same error codes on the other platforms where possible.
                // e.g. Widcomm doesn't match well, Bluetopia does.
                // http://32feet.codeplex.com/wikipage?title=Feature%20support%20table
                string reason;
                switch (ex.ErrorCode)
                {
                    case 10048: // SocketError.AddressAlreadyInUse
                        // RFCOMM only allow _one_ connection to a remote service from each device.
                        reason = "There is an existing connection to the remote Chat2 Service";
                        break;
                    case 10049: // SocketError.AddressNotAvailable
                        reason = "Chat2 Service not running on remote device";
                        break;
                    case 10064: // SocketError.HostDown
                        reason = "Chat2 Service not using RFCOMM (huh!!!)";
                        break;
                    case 10013: // SocketError.AccessDenied:
                        reason = "Authentication required";
                        break;
                    case 10060: // SocketError.TimedOut:
                        reason = "Timed-out";
                        break;
                    default:
                        reason = null;
                        break;
                }
                reason += " (" + ex.ErrorCode.ToString() + ") -- ";
                //
                var msg = "Bluetooth connection failed: " + ex.Message;
                msg = reason + msg;
                AddMessage(MessageSource.Error, msg);
                System.Windows.MessageBox.Show(msg);
            }
            catch (Exception ex)
            {
                var msg = "Bluetooth connection failed: " + ex.Message;
                AddMessage(MessageSource.Error, msg);
                System.Windows.MessageBox.Show(msg);
            }
        }

        private void StartListener()
        {
            var lsnr = new BluetoothListener(OurServiceClassId);
            lsnr.ServiceName = OurServiceName;
            lsnr.Start();
            _lsnr = lsnr;
            ThreadPool.QueueUserWorkItem(ListenerAccept_Runner, lsnr);
        }

        void ListenerAccept_Runner(object state)
        {
            var lsnr = (BluetoothListener)_lsnr;
            // We will accept only one incoming connection at a time. So just
            // accept the connection and loop until it closes.
            // To handle multiple connections we would need one threads for
            // each or async code.
            while (true)
            {
                var conn = lsnr.AcceptBluetoothClient();
                var peer = conn.GetStream();
                SetConnection(peer, false, conn.RemoteEndPoint);
                ReadMessagesToEnd(peer);
            }
        }
        #endregion

        #region Connection Set/Close
        private void SetConnection(Stream peerStream, bool outbound, BluetoothEndPoint remoteEndPoint)
        {
            if (_connWtr != null)
            {
                AddMessage(MessageSource.Error, "Already Connected!");
                return;
            }
            _closing = false;
            var connWtr = new StreamWriter(peerStream);
            connWtr.NewLine = "\r\n"; // Want CR+LF even on UNIX/Mac etc.
            _connWtr = connWtr;
            ClearScreen();
            AddMessage(MessageSource.Info,
                (outbound ? "Connected to " : "Connection from ")
                // Can't guarantee that the Port is set, so just print the address.
                // For more info see the docs on BluetoothClient.RemoteEndPoint.
                + remoteEndPoint.Address);
        }

        private void ConnectionCleanup()
        {
            _closing = true;
            var wtr = _connWtr;
            //_connStrm = null;
            _connWtr = null;
            if (wtr != null)
            {
                try
                {
                    wtr.Close();
                }
                catch (Exception ex)
                {
                    Debug.WriteLine("ConnectionCleanup close ex: " + ex.Message);
                }
            }
        }

        void BluetoothDisconnect()
        {
            AddMessage(MessageSource.Info, "Disconnecting");
            ConnectionCleanup();
        }
        #endregion

        #region Connection I/O
        private bool Send(string message)
        {
            if (_connWtr == null)
            {
                System.Windows.MessageBox.Show("No connection.");
                return false;
            }
            try
            {
                _connWtr.WriteLine(message);
                _connWtr.Flush();
                return true;
            }
            catch (Exception ex)
            {
                System.Windows.MessageBox.Show("Connection lost! (" + ex.Message + ")");
                ConnectionCleanup();
                return false;
            }
        }

        private void ReadMessagesToEnd_Runner(object state)
        {
            Stream peer = (Stream)state;
            ReadMessagesToEnd(peer);
        }

        private void ReadMessagesToEnd(Stream peer)
        {
            var rdr = new StreamReader(peer);
            while (true)
            {
                string line;
                try
                {
                    line = rdr.ReadLine();
                }
                catch (IOException ioex)
                {
                    if (_closing)
                    {
                        // Ignore the error that occurs when we're in a Read
                        // and _we_ close the connection.
                    }
                    else {
                        AddMessage(MessageSource.Error, "Connection was closed hard (read).  "
                            + ioex.Message);
                    }
                    break;
                }
                if (line == null)
                {
                    AddMessage(MessageSource.Info, "Connection was closed (read).");
                    break;
                }
                AddMessage(MessageSource.Remote, line);
            }//while
            ConnectionCleanup();
        }
        #endregion

        #region Chat Log
        private void ClearScreen()
        {
            Debug.WriteLine("BT MSG: CONNECTION CHANGED");
        }

        enum MessageSource
        {
            Local,
            Remote,
            Info,
            Error,
        }

        void AddMessage(MessageSource source, string message)
        {
            string prefix;
            switch (source)
            {
                case MessageSource.Local:
                    prefix = "Me: ";
                    break;
                case MessageSource.Remote:
                    prefix = "You: ";
                    break;
                case MessageSource.Info:
                    prefix = "Info: ";
                    break;
                case MessageSource.Error:
                    prefix = "Error: ";
                    break;
                default:
                    prefix = "???:";
                    break;
            }
            Debug.WriteLine("BT MSG " + prefix + message);
        }
        #endregion

        #region Skeleton WPF code

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Start bluetooth service
            StartBluetooth();

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

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
                this.sensor.SkeletonStream.Enable();

                // For debug: show camera
                this.sensor.ColorStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

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
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
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

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
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

        #endregion
    }
}