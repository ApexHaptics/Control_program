using Microsoft.Win32;
using System;
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
        /// For some reason it's different??? TODO
        /// </summary>
        const string PIDString = "2111";

        /// <summary>
        /// The VID of the Atmel device as specified in the specs (0x03EB)
        /// </summary>
        const string VIDString = "03EB";

        /// <summary>
        /// The baud rate of the device
        /// </summary>
        const int baudRate = 9600;

        /// <summary>
        /// The serial port which the Atmel is connected to.
        /// </summary>
        SerialPort serialPort;

        /// <summary>
        /// Threads to read and write from the Atmel
        /// </summary>
        Thread readThread, writeThread;

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
        }

        /// <summary>
        /// The thread which reads data from the Atmel
        /// </summary>
        private void ReadThread()
        {
            while (true)
            {
                try
                {
                    Console.WriteLine("Line rec: " + serialPort.ReadLine());
                }
                catch(TimeoutException)
                {
                    Console.WriteLine("heartbeat missed");
                }
            }
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
