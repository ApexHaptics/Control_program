﻿using Microsoft.Win32;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;

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
            public char tag;

            /// <summary>
            /// The packet type byte
            /// </summary>
            public char id;

            /// <summary>
            /// The data in the packet
            /// </summary>
            public String data;

            /// <summary>
            /// The message either read or written to comms
            /// </summary>
            public String message;

            /// <summary>
            /// Constructor when reading
            /// </summary>
            /// <param name="serialLine">The serial line read from the device</param>
            public RobotPacket(String serialLine)
            {
                int index = 0;
                if(serialLine[index] == '≥')
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
                data = serialLine.Substring(index);
                message = serialLine;

                data = data.Replace("\x10\x8A", "\x0A").Replace("\x10\x90", "\x10");
            }
        }

        /// <summary>
        /// Constructs a message to send to the Atmel
        /// </summary>
        /// <param name="synchronous">Is the message syncrhonous?</param>
        /// <param name="tag">The tag to send</param>
        /// <param name="id">The packet type</param>
        /// <param name="data">The data</param>
        /// <returns>A byte array to send</returns>
        private byte[] ConstructSendMessage(bool synchronous, char tag, char id, byte[] data)
        {
            byte[] constructArray = new byte[3+data.Length*2];
            int index = 0;
            if(!synchronous)
            {
                constructArray[index++] = (byte)'\xf2';
            }
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
        public SerialComms()
        {
            String portName = FindDevicePort();
            if(String.IsNullOrEmpty(portName))
            {
                Console.WriteLine("NO ATMEL DEVICE DETECTED");
                return;
            }

            serialPort = new SerialPort();
            serialPort.PortName = portName;
            serialPort.BaudRate = baudRate;
            serialPort.ReadTimeout = 6000;
            serialPort.WriteTimeout = 1500;
            // This is similar to extended ascii and will allow us to access all 256 values
            serialPort.Encoding = Encoding.GetEncoding(437);
            try {
                serialPort.Open();
            }
            catch(Exception e)
            {
                Console.WriteLine("COULDN'T OPEN SERIAL PORT: " + e.Message);
                return;
            }

            readThread = new Thread(ReadThread);
            readThread.Start();
            writeThread = new Thread(WriteThread);
            writeThread.Start();

            heartbeatTimer = new Timer(HeartbeatTimerExpiry);
            heartbeatTimer.Change(heartbeatDelay, heartbeatDelay);
        }

        /// <summary>
        /// Stops the read and write threads
        /// </summary>
        public void Stop()
        {
            _continue = false;
            unsentArrays.Add(null);
            readThread.Join();
            writeThread.Join();
        }

        public void HandleCommand(String command)
        {
            String[] parts = command.Split(' ');
            byte[] x, y, z;
            byte[] dataToSend;
            try {
                switch (parts[0])
                {
                    case "reset":
                        unsentArrays.Add(new byte[]{ (byte)'\n', (byte)'0', (byte)'M', (byte)'\n' });
                        break;
                    case "erase":
                        // We're not doing this yet
                        // unsentArrays.Add(new byte[] { (byte)'\n', (byte)'0', (byte)'O', (byte)'\n' });
                        break;
                    case "R":
                        x = parseFloatBytes(float.Parse(parts[1]));
                        y = parseFloatBytes(float.Parse(parts[2]));
                        z = parseFloatBytes(float.Parse(parts[3]));
                        dataToSend = new byte[x.Length + y.Length + z.Length];
                        x.CopyTo(dataToSend, 0);
                        y.CopyTo(dataToSend, x.Length);
                        z.CopyTo(dataToSend, x.Length + y.Length);
                        unsentArrays.Add(ConstructSendMessage(false, '\0', 'R', dataToSend));
                        break;
                    case "A":
                        dataToSend = new byte[1] { byte.Parse(parts[1]) };
                        unsentArrays.Add(ConstructSendMessage(false, '\0', 'A', dataToSend));
                        break;
                    case "Z":
                        x = parseFloatBytes(float.Parse(parts[1]));
                        y = parseFloatBytes(float.Parse(parts[2]));
                        z = parseFloatBytes(float.Parse(parts[3]));
                        dataToSend = new byte[x.Length + y.Length + z.Length];
                        x.CopyTo(dataToSend, 0);
                        y.CopyTo(dataToSend, x.Length);
                        z.CopyTo(dataToSend, x.Length + y.Length);
                        unsentArrays.Add(ConstructSendMessage(false, '\0', 'Z', dataToSend));
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
        /// Parses the bytes of a float to be sent over comms
        /// </summary>
        /// <param name="f">the float</param>
        /// <returns>the byte array to send</returns>
        private byte[] parseFloatBytes(float f)
        {
            byte[] returnArray = BitConverter.GetBytes(f);
            if (BitConverter.IsLittleEndian) return returnArray;

            // other-endian; reverse this portion of the data (4 bytes)
            byte tmp = returnArray[0];
            returnArray[0] = returnArray[3];
            returnArray[3] = tmp;
            tmp = returnArray[1];
            returnArray[1] = returnArray[2];
            returnArray[2] = tmp;

            return returnArray;
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
            String line;
            RobotPacket packet;
            while (_continue)
            {
                try
                {
                    line = serialPort.ReadLine();
                    if (String.IsNullOrEmpty(line) || line == "≥\0") continue;
                    if (line == "robo!" || line == "≥robo!")
                    {
                        heartbeatTimer.Change(heartbeatDelay, heartbeatDelay);
                        continue;
                    }
                    packet = new RobotPacket(line);

                    handleRobotPacket(packet);
                }
                catch(TimeoutException)
                {
                    Console.WriteLine("Serial read timeout");
                }
                catch
                {
                    Console.WriteLine("Serial read error");
                }
            }
        }

        private void handleRobotPacket(RobotPacket packet)
        {
            switch(packet.id)
            {
                case 'p':
                    Console.WriteLine("MCU print: " + packet.data);
                    // Ignoring for now due to large volume of prints
                    break;
                default:
                    Console.WriteLine("Unknown {0} packet rec. Tag:{1}, ID:{2}, data:{3}",
                        packet.synchronous ? "sync" : "async", packet.tag, packet.id, packet.data);
                    break;
            }
        }

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
