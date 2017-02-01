namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using InTheHand.Net;
    using InTheHand.Net.Sockets;
    using InTheHand.Windows.Forms;
    using System.Net.Sockets;
    using InTheHand.Net.Bluetooth;

    using System.Net;
    using System.IO;
    using System;
    using System.Diagnostics;
    using System.Threading;
    using System.Windows.Forms;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;

    /// <summary>
    /// A class to handle all bluetooh communications
    /// </summary>
    class BluetoothService
    {

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
        public void StartBluetooth()
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
        public bool Send(string message)
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
    }
}
