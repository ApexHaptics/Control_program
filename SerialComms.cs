using Microsoft.Win32;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Controls;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class SerialComms
    {
        /// <summary>
        /// The PID of the Atmel device as specified in the specs (0x6124)
        /// </summary>
        const string PIDString = "6124";

        /// <summary>
        /// The VID of the Atmel device as specified in the specs (0x03EB)
        /// </summary>
        const string VIDString = "03EB";

        /// <summary>
        /// The baud rate of the device
        /// The highest standard rate found here: http://digital.ni.com/public.nsf/allkb/D37754FFA24F7C3F86256706005B9BE7
        /// </summary>
        const int baudRate = 115200;

        /// <summary>
        /// The serial port which the Atmel is connected to.
        /// </summary>
        SerialPort serialPort;

        /// <summary>
        /// Threads to read and write from the Atmel
        /// </summary>
        Thread readThread, writeThread;

        /// <summary>
        /// The packets which are yet unsent
        /// </summary>
        BlockingCollection<byte[]> unsentArrays = new BlockingCollection<byte[]>(new ConcurrentQueue<byte[]>());

        /// <summary>
        /// Handlers for packet responses. Each sender must specify a handler, which may be null
        /// The string passed will be the packet data. This can be transformed into a byte array through BlockCopy operations
        /// Ex: delegate(string s){ Console.WriteLine("DELEGATE SUCESSFULLY INVOKED"); }
        /// </summary>
        BlockingCollection<Action<byte[]>> pendingActions = new BlockingCollection<Action<byte[]>>(new ConcurrentQueue<Action<byte[]>>());

        /// <summary>
        /// A timer to make sure a heartbeat is received every second
        /// </summary>
        Timer heartbeatTimer;

        /// <summary>
        /// The amount of milliseconds to wait before the heartbeat timer expires
        /// For reference the heartbeats should arrive every 1000ms
        /// </summary>
        const int heartbeatDelay = 5000;

        /// <summary>
        /// If false, the read and write threads will terminate
        /// </summary>
        bool _continue = true;

        /// <summary>
        /// A delegate type for handling a kinematic position update from the robot
        /// </summary>
        public delegate void KinPosUpdateHandler(double[] position);

        /// <summary>
        /// The event fired when a new kinematic position is sent from the robot
        /// </summary>
        public event KinPosUpdateHandler kinPosUpdated;

        /// <summary>
        /// A delegate type for handling a robot enabled state update
        /// </summary>
        public delegate void EnabledStateHandler(bool isEnabled);

        /// <summary>
        /// The event fired when a the robot reports that it has been successfully enabled or disabled
        /// </summary>
        public event EnabledStateHandler enabledStateUpdated;

        /// <summary>
        /// Whether the robot's comms are enabled
        /// </summary>
        private bool robotIsEnabled = false;

        /// <summary>
        /// The button which enables/disables the controller
        /// </summary>
        Button enableButton;

        /// <summary>
        /// A packet received from the robot
        /// </summary>
        public struct RobotPacket
        {
            /// <summary>
            /// Whether the packet is asynchronous or synchronous.
            /// So far there are only two types but may need changing
            /// </summary>
            public bool synchronous;

            /// <summary>
            /// A byte representing the packet number
            /// </summary>
            public byte tag;

            /// <summary>
            /// The packet type byte
            /// </summary>
            public byte id;

            /// <summary>
            /// The data in the packet
            /// </summary>
            public byte[] data;

            /// <summary>
            /// The message either read or written to comms
            /// </summary>
            public byte[] message;

            /// <summary>
            /// Constructor when reading
            /// </summary>
            /// <param name="serialLine">The serial line read from the device</param>
            public RobotPacket(byte[] serialLine)
            {
                int index = 0;
                if(serialLine[index] == 0xF2)
                {
                    synchronous = false;
                    index++;
                }
                else
                {
                    synchronous = true;
                }
                tag = serialLine[index++];
                id = serialLine[index++];
                List<byte> dataList = new List<byte>();
                for(int i = index; i < serialLine.Length; i++)
                {
                    if(serialLine[i] == 0x10 && serialLine.Length > i+1)
                    {
                        if(serialLine[i+1] == 0x8A)
                        {
                            i++;
                            dataList.Add(0x0A);
                            continue;
                        }
                        else if (serialLine[i + 1] == 0x90)
                        {
                            i++;
                            dataList.Add(0x10);
                            continue;
                        }
                    }
                    dataList.Add(serialLine[i]);
                }
                message = serialLine;
                data = dataList.ToArray();
            }
        }

        /// <summary>
        /// Constructs a message to send to the Atmel
        /// </summary>
        /// <param name="tag">The tag to send</param>
        /// <param name="id">The packet type</param>
        /// <param name="data">The data</param>
        /// <returns>A byte array to send</returns>
        private byte[] ConstructSendMessage(char tag, char id, byte[] data)
        {
            byte[] constructArray = new byte[3+data.Length*2];
            int index = 0;
            constructArray[index++] = (byte) tag;
            constructArray[index++] = (byte) id;
            foreach(byte dataByte in data)
            {
                if(dataByte == (byte) '\x0A')
                {
                    constructArray[index++] = (byte)'\x10';
                    constructArray[index++] = (byte)'\x8A';
                }
                else if (dataByte == (byte)'\x10')
                {
                    constructArray[index++] = (byte)'\x10';
                    constructArray[index++] = (byte)'\x90';
                }
                else
                {
                    constructArray[index++] = dataByte;
                }
            }

            byte[] returnArray = new byte[index];
            Array.Copy(constructArray, 0, returnArray, 0, index);

            return returnArray;
        }

        /// <summary>
        /// Constructor for the SerialComms class
        /// </summary>
        /// <param name="enableButton">The button to enable/disable the controller</param>
        public SerialComms(Button enableButton)
        {
            this.enableButton = enableButton;
            ThreadPool.QueueUserWorkItem(delegate
            {
                string portName;
                do
                {
                    portName = FindDevicePort();
                    Console.WriteLine("NO ATMEL DEVICE DETECTED");
                    Thread.Sleep(5000);
                } while (string.IsNullOrEmpty(portName));

                    serialPort = new SerialPort();
                serialPort.PortName = portName;
                serialPort.BaudRate = baudRate;
                serialPort.ReadTimeout = 6000;
                serialPort.WriteTimeout = 1500;
                // This is similar to extended ascii and will allow us to access all 256 values
                serialPort.Encoding = Encoding.GetEncoding(437);
                try
                {
                    serialPort.Open();
                }
                catch (Exception e)
                {
                    Console.WriteLine("COULDN'T OPEN SERIAL PORT: " + e.Message);
                    return;
                }

                heartbeatTimer = new Timer(HeartbeatTimerExpiry);
                heartbeatTimer.Change(heartbeatDelay, heartbeatDelay);

                readThread = new Thread(ReadThread);
                readThread.Start();
                writeThread = new Thread(WriteThread);
                writeThread.Start();
            });
        }

        /// <summary>
        /// Stops the read and write threads
        /// </summary>
        public void Stop()
        {
            _continue = false;
            unsentArrays.Add(null);
            if (readThread != null)
            {
                readThread.Join();
            }
            if (writeThread != null)
            {
                writeThread.Join();
            }
        }

        public void HandleCommand(String command)
        {
            String[] parts = command.Split(' ');
            try {
                switch (parts[0])
                {
                    case "reset":
                        unsentArrays.Add(new byte[]{ (byte)'\n', (byte)'0', (byte)'M', (byte)'\n' });
                        pendingActions.Add(null);
                        break;
                    case "erase":
                        // We're not doing this yet
                        // unsentArrays.Add(new byte[] { (byte)'\n', (byte)'0', (byte)'O', (byte)'\n' });
                        break;
                    case "R":
                        SetTargetPosition(float.Parse(parts[1]), float.Parse(parts[2]), float.Parse(parts[3]));
                        break;
                    case "A":
                        byte[] dataToSend = new byte[1] { byte.Parse(parts[1]) };
                        unsentArrays.Add(ConstructSendMessage('\0', 'A', dataToSend));
                        pendingActions.Add(null);
                        break;
                    case "Z":
                        SetImpedance(float.Parse(parts[1]), float.Parse(parts[2]), float.Parse(parts[3]));
                        break;
                    case "E":
                    case "Enable":
                        if (!enableButton.IsEnabled) break;
                        enableButton.IsEnabled = false;
                        unsentArrays.Add(ConstructSendMessage('\0', 'E', new byte[0]));
                        pendingActions.Add(enablerHandler);
                        enableButton.Background = System.Windows.Media.Brushes.DarkGray;
                        break;
                    case "D":
                    case "Disable":
                        if (!enableButton.IsEnabled) break;
                        enableButton.IsEnabled = false;
                        unsentArrays.Add(ConstructSendMessage('\0', 'D', new byte[0]));
                        pendingActions.Add(enablerHandler);
                        enableButton.Background = System.Windows.Media.Brushes.DarkGray;
                        break;
                    default:
                        Console.WriteLine("Comms unrecognized command");
                        break;
                }
            }
            catch
            {
                Console.WriteLine("Comms errorneous command");
            }
        }

        /// <summary>
        /// Sets a target position for the end effector
        /// </summary>
        /// <param name="x">Position x in meters</param>
        /// <param name="y">Position y in meters</param>
        /// <param name="z">Position z in meters</param>
        public void SetTargetPosition(float x, float y, float z)
        {
            byte[] x_bytes = parseFloatBytes(x);
            byte[] y_bytes = parseFloatBytes(y);
            byte[] z_bytes = parseFloatBytes(z);
            byte[] dataToSend = new byte[x_bytes.Length + y_bytes.Length + z_bytes.Length];
            x_bytes.CopyTo(dataToSend, 0);
            y_bytes.CopyTo(dataToSend, x_bytes.Length);
            z_bytes.CopyTo(dataToSend, x_bytes.Length + y_bytes.Length);
            unsentArrays.Add(ConstructSendMessage('\0', 'R', dataToSend));
            pendingActions.Add(null);
        }

        /// <summary>
        /// Sets the impedance of the end effector
        /// </summary>
        /// <param name="m">Simulated mass</param>
        /// <param name="b">Damping constant</param>
        /// <param name="k">Spring constant</param>
        public void SetImpedance(float m, float b, float k)
        {
            byte[] m_bytes = parseFloatBytes(m);
            byte[] b_bytes = parseFloatBytes(b);
            byte[] k_bytes = parseFloatBytes(k);
            byte[] dataToSend = new byte[m_bytes.Length + b_bytes.Length + k_bytes.Length];
            m_bytes.CopyTo(dataToSend, 0);
            b_bytes.CopyTo(dataToSend, m_bytes.Length);
            k_bytes.CopyTo(dataToSend, m_bytes.Length + b_bytes.Length);
            unsentArrays.Add(ConstructSendMessage('\0', 'Z', dataToSend));
            pendingActions.Add(null);
        }

        /// <summary>
        /// Handler for enable/disable commands
        /// </summary>
        /// <param name="b">Unused</param>
        void enablerHandler(byte[] b)
        {
            string buttonString;
            System.Windows.Media.Brush buttonColor;
            robotIsEnabled = !robotIsEnabled;

            enabledStateUpdated(robotIsEnabled);
            if(robotIsEnabled)
            {
                buttonString = "Disable Controller";
                buttonColor = System.Windows.Media.Brushes.DarkRed;
            }
            else
            {
                buttonString = "Enable Controller";
                buttonColor = System.Windows.Media.Brushes.DarkGreen;
            }
            
            enableButton.Dispatcher.BeginInvoke(new Action(() => {
                enableButton.Content = buttonString;
                enableButton.Background = buttonColor;
                enableButton.IsEnabled = true;
            }));
        }

        /// <summary>
        /// Parses the bytes of a float to be sent over comms
        /// </summary>
        /// <param name="f">the float</param>
        /// <returns>the byte array to send</returns>
        private byte[] parseFloatBytes(float f)
        {
            byte[] returnArray = BitConverter.GetBytes(f);
            if (BitConverter.IsLittleEndian) return returnArray;

            Array.Reverse(returnArray);
            return returnArray;
        }

        /// <summary>
        /// Parses a float from bytes received over comms
        /// </summary>
        /// <param name="array">The byte array</param>
        /// <param name="index">The index to start at</param>
        /// <returns>The float it represents</returns>
        private float parseFloatBytes(byte[] array, int index)
        {
            if (BitConverter.IsLittleEndian)
            {
                return BitConverter.ToSingle(array, index);
            }

            byte[] subArrray = new byte[] { array[index + 3],
                array[index + 2], array[index + 1], array[index] };

            Array.Reverse(subArrray);
            return BitConverter.ToSingle(subArrray, 0);
        }

        /// <summary>
        /// The thread which writes data to the Atmel
        /// </summary>
        private void WriteThread()
        {
            byte[] arrayToWrite;
            while (_continue)
            {
                try
                {
                    arrayToWrite = unsentArrays.Take(); // blocking
                    if (arrayToWrite == null) continue; // Sentinel value
                    serialPort.Write("\n");
                    serialPort.Write(arrayToWrite, 0, arrayToWrite.Length);
                    serialPort.Write("\n");
                }
                catch (TimeoutException)
                {
                    Console.WriteLine("Serial write timeout");
                }
                catch
                {
                    Console.WriteLine("Serial write error");
                }
            }
        }

        /// <summary>
        /// The thread which reads data from the Atmel
        /// </summary>
        private void ReadThread()
        {
            RobotPacket packet;
            List<byte> emptyLine = (new byte[] { 0xF2, 0 }).ToList();
            List<byte> roboLine = (new byte[] { 0xF2, (byte)'r', (byte)'o',
                (byte)'b', (byte)'o', (byte)'!'}).ToList();
            while (_continue)
            {
                try
                {
                    List<byte> line = new List<byte>();
                    byte b = (byte)serialPort.ReadByte();
                    while(b != '\n')
                    {
                        line.Add(b);
                        b = (byte)serialPort.ReadByte();
                    }
                    if (line.Count == 0 || line.SequenceEqual(emptyLine)) continue;
                    if (line.SequenceEqual(roboLine))
                    {
                        heartbeatTimer.Change(heartbeatDelay, heartbeatDelay);
                        continue;
                    }
                    packet = new RobotPacket(line.ToArray());

                    if(packet.synchronous)
                    {
                        Action<byte[]> action = pendingActions.Take();
                        if(action != null)
                        {
                            action(packet.data);
                        }
                    }
                    else
                    {
                        handleRobotPacket(packet);
                    }
                }
                catch(TimeoutException)
                {
                    Console.WriteLine("Serial read timeout");
                }
                catch(Exception e)
                {
                    Console.WriteLine("Serial read error: " + e.Message);
                }
            }
        }

        /// <summary>
        /// A function to handle an asynchronour robot packet
        /// </summary>
        /// <param name="packet">The packet from the robot</param>
        private void handleRobotPacket(RobotPacket packet)
        {
            switch(packet.id)
            {
                case (byte)'p':
                    // Console.WriteLine("MCU print: " + packet.data);
                    // Ignoring for now due to large volume of prints
                    break;
                case (byte)'A':
                    // Packet content should be 3 floats theta1, theta2, theta3
                    float theta1 = parseFloatBytes(packet.data, 0);
                    float theta2 = parseFloatBytes(packet.data, 4);
                    float theta3 = parseFloatBytes(packet.data, 8);
                    double[] position = Physics.physics_fkin(theta1, theta2, theta3);
                    kinPosUpdated(position);
                    break;
                default:
                    Console.WriteLine("Unknown {0} packet rec. Tag:{1}, ID:{2}, data:{3}",
                        packet.synchronous ? "sync" : "async",
                        packet.tag == '\0' ? "null" : packet.tag.ToString(),
                        packet.id, packet.data);
                    break;
            }
        }

        /// <summary>
        /// A function which reports when heartbeat packets timeout
        /// </summary>
        /// <param name="state">Unused</param>
        private void HeartbeatTimerExpiry(object state)
        {
            Console.WriteLine("MCU heartbeat missed");
        }

        /// <summary>
        /// Finds the port of the device
        /// </summary>
        /// <returns>The serial port name if found, String.Empty otherwise</returns>
        private String FindDevicePort()
        {
            List<string> names = ComPortNames(VIDString, PIDString);
            if (names.Count > 0)
            {
                foreach (String s in SerialPort.GetPortNames())
                {
                    if (names.Contains(s))
                        return s;
                }
            }
            return String.Empty;
        }

        /// <summary>
        /// Compile an array of COM port names associated with given VID and PID.
        /// This may return disconnected devices.
        /// </summary>
        /// <param name="VID"></param>
        /// <param name="PID"></param>
        /// <returns></returns>
        private List<string> ComPortNames(String VID, String PID)
        {
            String pattern = String.Format("^VID_{0}.PID_{1}", VID, PID);
            Regex _rx = new Regex(pattern, RegexOptions.IgnoreCase);
            List<string> comports = new List<string>();
            RegistryKey rk1 = Registry.LocalMachine;
            RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");
            foreach (String s3 in rk2.GetSubKeyNames())
            {
                RegistryKey rk3 = rk2.OpenSubKey(s3);
                foreach (String s in rk3.GetSubKeyNames())
                {
                    if (_rx.Match(s).Success)
                    {
                        RegistryKey rk4 = rk3.OpenSubKey(s);
                        foreach (String s2 in rk4.GetSubKeyNames())
                        {
                            RegistryKey rk5 = rk4.OpenSubKey(s2);
                            RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");
                            comports.Add((string)rk6.GetValue("PortName"));
                        }
                    }
                }
            }
            return comports;
        }
    }
}
