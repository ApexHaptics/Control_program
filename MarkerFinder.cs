using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;

using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Windows.Media;
using System.Windows.Interop;
using System.Windows;
using System.Windows.Media.Imaging;
using System.IO;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class MarkerFinder
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Width of colour image
        /// </summary>
        private const int cImgWidth = 1280;

        /// <summary>
        /// Height of colour image
        /// </summary>
        private const int cImgHeight = 960;

        /// <summary>
        /// Marker finding will only run one in throttleFinding times
        /// </summary>
        //private const int throttleFinding = 1;

        /// <summary>
        /// Drawing group for skeleton rendering marker output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// The dictionary used to detect markers
        /// </summary>
        private readonly Dictionary dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);

        /// <summary>
        /// Brush used to draw marker center point
        /// </summary>
        private readonly System.Windows.Media.Brush[] cornerBrushes = { System.Windows.Media.Brushes.Blue,
            System.Windows.Media.Brushes.Red,
            System.Windows.Media.Brushes.Yellow,
            System.Windows.Media.Brushes.Green };

        /// <summary>
        /// Thickness of marker center ellipse
        /// </summary>
        private const double CornerThickness = 2;

        /// <summary>
        /// The camera parameters found from the camera calibration
        /// </summary>
        private Matrix<double> cameraMatrix = new Matrix<double>(3, 3);

        /// <summary>
        /// The distortion parameters found from the camera calibration
        /// </summary>
        private Matrix<double> distortionParameters = new Matrix<double>(1, 5);

        /// <summary>
        /// The last time of the last camera update
        /// </summary>
        private DateTime lastUpdateTime = new DateTime(0);

        /// <summary>
        /// The length of a marker edge (in m) used for scaling translation vectors
        /// </summary>
        private const float MarkerWidth = 0.10f;

        /// <summary>
        /// The angle which the marker must be tilted less than
        /// </summary>
        private const double ThresholdMarkerAngle = Math.PI / 4;

        /// <summary>
        /// What the marker IDs actually correspond to
        /// </summary>
        private enum MarkerTypes
        {
            HeadLeft = 0,
            HeadCenter = 1,
            HeadRight = 2,
            EndEffector = 3,
        }

        /// <summary>
        /// How much we are lowpassing the calibration by
        /// </summary>
        private double calibAlpha = 0.1;

        /// <summary>
        /// Lowpass filters for all calibration parameters (translation vector XYZ, rotation vector XYZ)
        /// </summary>
        private List<LowpassFilter> robotCalibrationFilters = new List<LowpassFilter>();

        /// <summary>
        /// The window for displaying the markers and colour image
        /// </summary>
        ColourImageWindow colourWindow = new ColourImageWindow();

        /// <summary>
        /// Constructor for the MarkerFinder class
        /// </summary>
        /// <param name="drawingGroup">Where the skeleton will be drawn onto</param>
        public MarkerFinder(DrawingGroup drawingGroup)
        {
            this.drawingGroup = drawingGroup;

            //From running OCVKinectCameraCalib we get the following:
            //Intrinsic Calculation Error: 0.715600135211483
            //Results:
            //Camera matrix: 1086.56022934489,0,638.973519919036,0,1085.81336040516,479.725172086703,0,0,1,
            //Distortion Parameters: 0.107123870820562,0,0,0,0,6.95232994533967E-310,0,6.95232994713451E-310,0,7.64923658912117E+232,0,5.28496633257694E-308,2.12199579145934E-314,8.12198220026745E-312,
            double[] cameraMatrixArray0 = { 1086.56022934489, 0, 638.973519919036, };
            double[] cameraMatrixArray1 = { 0, 1085.81336040516, 479.725172086703, };
            double[] cameraMatrixArray2 = { 0, 0, 1 };
            double[][] cameraMatrixArray = { cameraMatrixArray0, cameraMatrixArray1, cameraMatrixArray2 };
            double[] distortionParametersArray = { 0.107123870820562, 0, 0, 0, 0, 6.95232994533967E-310, 0, 6.95232994713451E-310, 0, 7.64923658912117E+232, 0, 5.28496633257694E-308, 2.12199579145934E-314, 8.12198220026745E-312, };
            for (int i = 0; i < cameraMatrixArray.Length; i++)
            {
                for (int j = 0; j < cameraMatrixArray[i].Length; j++)
                {
                    cameraMatrix.Data[i, j] = cameraMatrixArray[i][j];
                }
            }
            for (int i = 0; i < distortionParameters.Width; i++)
            {
                distortionParameters.Data[0,i] = distortionParametersArray[i];
            }

            colourWindow.Show();

            // This code generates all the test markers. Uncomment to generate
            //for (int i = 0; i < dictionaryCount; i++)
            //{
            //    Mat markerImage = new Mat();
            //    ArucoInvoke.DrawMarker(dictionary, i, 200, markerImage);
            //    markerImage.Save("Marker" + i + ".png");
            //}
        }

        /// <summary>
        /// Returns the marker locations in a new colour frame. All positions in m
        /// </summary>
        /// <param name="e">The colour frame data passed</param>
        /// <param name="sender">object sending the event</param>
        /// <param name="idList">a list to be populated with the marker ids placed in the return</param>
        /// <param name="rotationVectors">a list to be populated with found rotation vectors</param>
        /// <param name="translationVectors">a list to be populated with found translation vectors</param>
        /// <param name="deltaT">will be populated with the delta time since the last update (in ms)</param>
        /// <param name="headPos">will be populated with the averaged position vector of the head (x,y,z)</param>
        /// <param name="headRotMatrix">The estimated rotation matrix of the head [x,y,z] axis vectors</param>
        /// <param name="robotPos">will be populated with the position vector of the robot (x,y,z)</param>
        /// <param name="robotRotMatrix">The estimated rotation matrix of the robot [x,y,z] axis vectors</param>
        /// <returns>The number of markers found</returns>
        public int FindMarkers(ColorImageFrameReadyEventArgs e, List<int> idList, List<double[]> rotationVectors,
            List<double[]> translationVectors, ref double deltaT, List<double> headPos, List<double> headRotMatrix,
            List<double> robotPos, List<double> robotRotMatrix)
        {
            // Stopwatch code: put the last 2 seconds before the good return
            //System.Diagnostics.Stopwatch stopwatch = System.Diagnostics.Stopwatch.StartNew(); //creates and start the instance of Stopwatch
            //stopwatch.Stop();
            //Console.WriteLine("Marker Latency: " + stopwatch.ElapsedMilliseconds);

            using (VectorOfInt ids = new VectorOfInt())
            using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
            using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame == null) return 0;

                DetectorParameters p = DetectorParameters.GetDefault();
                p.DoCornerRefinement = true;
                p.AdaptiveThreshConstant = 7;
                p.AdaptiveThreshWinSizeStep = 2;
                p.AdaptiveThreshWinSizeMin = 90;
                p.AdaptiveThreshWinSizeMax = 90;
                Bitmap frameBitmap = ImageToBitmap(frame);
                colourWindow.setImageBitmapSource(frameBitmap);
                frameBitmap.RotateFlip(RotateFlipType.RotateNoneFlipX); // The Kinect sems to flip the image
                Image<Bgr, byte> imageFromKinect = new Image<Bgr, byte>(frameBitmap);

                // Manual thresholding code
                //Image<Gray, Byte> grayFrame = imageFromKinect.Convert<Gray, Byte>();
                //Image<Gray, Byte> grayFrameThresh = new Image<Gray, Byte>(grayFrame.Size);
                //CvInvoke.Threshold(grayFrame, grayFrameThresh, colourWindow.thresholdValue,255,Emgu.CV.CvEnum.ThresholdType.Binary);
                //Bitmap threshBitmap = grayFrameThresh.ToBitmap();
                //colourWindow.setImageBitmapSource(threshBitmap);

                ArucoInvoke.DetectMarkers(imageFromKinect, dictionary, corners, ids, p, rejected);
                PointF[][] cornersArray = corners.ToArrayOfArray();

                if (ids.Size == 0)
                {
                    // Clear the screen
                    using (DrawingContext dc = colourWindow.markerDrawingGroup.Open())
                    {
                        dc.DrawRectangle(System.Windows.Media.Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                    }
                    return 0;
                }

                bool allBadIds = true;
                for (int i = 0; i < ids.Size; i++)
                {
                    if(Enum.GetName(typeof(MarkerTypes),ids[i]) != null)
                    {
                        allBadIds = false;
                        break;
                    }
                }
                if (allBadIds) return 0;

                DateTime tempNow = DateTime.Now;
                if(lastUpdateTime.Ticks == 0)
                {
                    deltaT = 0;
                }
                else
                {
                    deltaT = tempNow.Subtract(lastUpdateTime).TotalMilliseconds;
                }
                lastUpdateTime = tempNow;

                // Populate the marker ids
                for (int i = 0; i < ids.Size; i++)
                {
                    idList.Add(ids[i]);
                }


                using (Mat rvecs = new Mat())
                using (Mat tvecs = new Mat())
                {
                    try
                    {
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, MarkerWidth, cameraMatrix, distortionParameters, rvecs, tvecs);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Failed to estimate marker pose: " + ex.Message);
                    }
                    for (int i = 0; i < ids.Size; i++)
                    {
                        using (Mat rvecMat = rvecs.Row(i))
                        using (Mat tvecMat = tvecs.Row(i))
                        using (Mat transMat = new Mat())
                        {
                            double[] rvecValues = new double[3];
                            rvecMat.CopyTo(rvecValues);
                            rotationVectors.Add(rvecValues);
                            double[] tvecValues = new double[3];
                            tvecMat.CopyTo(tvecValues);
                            translationVectors.Add(tvecValues);

                            // Calculate position with offset
                            Matrix<double> offset = new Matrix<double>(3, 1);
                            switch ((MarkerTypes)ids[i])
                            {
                                case MarkerTypes.HeadLeft:
                                    offset[0, 0] = -0.072;
                                    offset[1, 0] = 0;
                                    offset[2, 0] = -0.106;
                                    break;
                                case MarkerTypes.HeadCenter:
                                    offset[0, 0] = 0;
                                    offset[1, 0] = 0;
                                    offset[2, 0] = -0.023;
                                    break;
                                case MarkerTypes.HeadRight:
                                    offset[0, 0] = 0.072;
                                    offset[1, 0] = 0;
                                    offset[2, 0] = -0.106;
                                    break;
                                case MarkerTypes.EndEffector:
                                    // TODO: Find end effector offset
                                    offset[0, 0] = 0;
                                    offset[1, 0] = 0;
                                    offset[2, 0] = 0;
                                    break;
                                default:
                                    continue;
                            }

                            CvInvoke.Rodrigues(rvecMat, transMat);
                            Matrix<double> transMatrix = new Matrix<double>(transMat.Width, transMat.Height);
                            transMat.CopyTo(transMatrix);
                            Matrix<double> position = new Matrix<double>(translationVectors[i]);
                            position = position + transMatrix * offset;

                            // End effector calibration logic
                            if ((MarkerTypes)ids[i] == MarkerTypes.EndEffector)
                            {
                                bool isCalibrating = robotCalibrationFilters.Count > 0;
                                if(colourWindow.shouldCalibrate)
                                {
                                    if(!isCalibrating)
                                    {
                                        for(int j = 0; j < 6; j++)
                                        {
                                            robotCalibrationFilters.Add(new LowpassFilter());
                                        }
                                        robotCalibrationFilters[0].Filter(position.Data[0, 0], calibAlpha);
                                        robotCalibrationFilters[1].Filter(position.Data[1, 0], calibAlpha);
                                        robotCalibrationFilters[2].Filter(position.Data[2, 0], calibAlpha);
                                        robotCalibrationFilters[3].Filter(rvecValues[0], calibAlpha);
                                        robotCalibrationFilters[4].Filter(rvecValues[1], calibAlpha);
                                        robotCalibrationFilters[5].Filter(rvecValues[2], calibAlpha);
                                    }
                                }
                                else if (isCalibrating && !colourWindow.shouldCalibrate) {
                                    // Calibration completed
                                    robotPos.Add(robotCalibrationFilters[0].Filter(position.Data[0, 0], calibAlpha));
                                    robotPos.Add(robotCalibrationFilters[1].Filter(position.Data[1, 0], calibAlpha));
                                    robotPos.Add(robotCalibrationFilters[2].Filter(position.Data[2, 0], calibAlpha));

                                    Matrix<double> rvecForCalibration = new Matrix<double>(3,1);
                                    rvecForCalibration.Data[0, 0] = robotCalibrationFilters[3].Filter(rvecValues[0], calibAlpha);
                                    rvecForCalibration.Data[1, 0] = robotCalibrationFilters[4].Filter(rvecValues[1], calibAlpha);
                                    rvecForCalibration.Data[2, 0] = robotCalibrationFilters[5].Filter(rvecValues[2], calibAlpha);

                                    CvInvoke.Rodrigues(rvecForCalibration, transMat);
                                    double[] robotRotMatrixArray = new double[] {
                                        transMatrix.Data[0, 0], transMatrix.Data[0, 1], transMatrix.Data[0, 2],
                                        transMatrix.Data[1, 0], transMatrix.Data[1, 1], transMatrix.Data[1, 2],
                                        transMatrix.Data[2, 0], transMatrix.Data[2, 1], transMatrix.Data[2, 2],
                                    };
                                    robotRotMatrix.AddRange(robotRotMatrixArray);
                                    robotCalibrationFilters = new List<LowpassFilter>(); // Zero it again
                                    continue;
                                }
                            }

                            // Calculate view angle - Only for head markers
                            double offset_angle;
                            switch ((MarkerTypes)ids[i]) // Find the horizontal offset angle of the 
                            {
                                case MarkerTypes.HeadLeft:
                                    offset_angle = Math.PI / -2;
                                    break;
                                case MarkerTypes.HeadCenter:
                                    offset_angle = 0;
                                    break;
                                case MarkerTypes.HeadRight:
                                    offset_angle = Math.PI / 2;
                                    break;
                                default:
                                    continue;
                            }
                            Matrix<double> trans_x = transMatrix.GetCol(0);
                            Matrix<double> trans_z = transMatrix.GetCol(2);

                            double angleOff = Math.Acos(trans_z.Data[0, 2] / VecMagnitude(trans_z));
                            if (angleOff < ThresholdMarkerAngle) break;
                            
                            headPos.Add(position.Data[0, 0]);
                            headPos.Add(position.Data[1, 0]);
                            headPos.Add(position.Data[2, 0]);

                            // Manual rotation matrix
                            Matrix<double> new_x_axis = Math.Cos(offset_angle) * trans_x + Math.Sin(offset_angle) * trans_z;
                            Matrix<double> new_z_axis = Math.Cos(offset_angle) * trans_z - Math.Sin(offset_angle) * trans_x;

                            double[] outputRotMatrix = new double[] {
                                new_x_axis.Data[0, 0], transMatrix.Data[0, 1], new_z_axis.Data[0, 0],
                                new_x_axis.Data[1, 0], transMatrix.Data[1, 1], new_z_axis.Data[1, 0],
                                new_x_axis.Data[2, 0], transMatrix.Data[2, 1], new_z_axis.Data[2, 0],
                            };
                            headRotMatrix.AddRange(outputRotMatrix);
                        }
                    }
                }

                // Draw the markers
                using (DrawingContext dc = colourWindow.markerDrawingGroup.Open())
                {
                    dc.DrawRectangle(System.Windows.Media.Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                    for (int i = 0; i < cornersArray.Length; i++)
                    {
                        if (!Enum.IsDefined(typeof(MarkerTypes), ids[i])) continue;

                        PointF[] markerArray = cornersArray[i];
                        for (int j = 0; j < markerArray.Length; j++)
                        {
                            // From front view, top right = blue = markerarray[0]. Rest are CCW
                            System.Windows.Point cornerPoint = new System.Windows.Point((int)markerArray[j].X/2, (int)markerArray[j].Y/2);
                            dc.DrawEllipse(
                                this.cornerBrushes[j],
                                null,
                                cornerPoint,
                                CornerThickness,
                                CornerThickness);

                            // Redo mirroring
                            markerArray[j].X = cImgWidth - markerArray[j].X;
                        }
                    }
                }
                return ids.Size;
            }
        }

        /// <summary>
        /// Helper method to change kinect images to bitmaps
        /// http://stackoverflow.com/questions/10848190/convert-kinect-colorimageframe-to-bitmap
        /// </summary>
        /// <param name="Image">The kinect image frame</param>
        /// <returns>The image data as a bitmap</returns>
        Bitmap ImageToBitmap(ColorImageFrame Image)
        {
            byte[] pixeldata = new byte[Image.PixelDataLength];
            Image.CopyPixelDataTo(pixeldata);
            Bitmap bmap = new Bitmap(Image.Width, Image.Height, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
            BitmapData bmapdata = bmap.LockBits(
                new Rectangle(0, 0, Image.Width, Image.Height),
                ImageLockMode.WriteOnly,
                bmap.PixelFormat);
            IntPtr ptr = bmapdata.Scan0;
            Marshal.Copy(pixeldata, 0, ptr, Image.PixelDataLength);
            bmap.UnlockBits(bmapdata);
            return bmap;
        }

        /// <summary>
        /// Finds the magnitude of a 3D vector
        /// </summary>
        /// <param name="mat">Input matrix (3d vertical)</param>
        /// <returns>The matrix's magnitude</returns>
        double VecMagnitude(Matrix<double> mat)
        {
            double squaredSum = Math.Pow(mat.Data[0, 0], 2) + Math.Pow(mat.Data[0, 1], 2) + Math.Pow(mat.Data[0, 2], 2);
            return Math.Sqrt(squaredSum);
        }
    }
}
