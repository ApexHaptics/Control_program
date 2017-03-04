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
        private const int throttleFinding = 12;

        /// <summary>
        /// Total number of frames processed
        /// </summary>
        private int framesProcessed = 0;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// The dictionary used to detect markers
        /// </summary>
        private readonly Dictionary dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_100);

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
        private const double CornerThickness = 5;

        private PointF[] objCorners = new PointF[]{new PointF(-4, -4),
            new PointF(4, -4),
            new PointF(4, 4),
            new PointF(-4, 4)};

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
        private DateTime lastUpdateTime = new DateTime();

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
            //Camera matrix: 1003.51217273555,0,639.744767107202,0,999.013101464053,479.683396908507,0,0,1,
            //Distortion Parameters: 0.079800650654781,0,2.12199579096527E-314,0,1.26767782840851E-311,1.26763894007465E-311,0,0,0,1.26739902687602E-311,0,0,4.9868923567144E-316,0,
            double[] cameraMatrixArray0 = { 1003.51217273555, 0, 639.744767107202 };
            double[] cameraMatrixArray1 = { 0, 999.013101464053, 479.683396908507 };
            double[] cameraMatrixArray2 = { 0, 0, 1 };
            double[][] cameraMatrixArray = { cameraMatrixArray0, cameraMatrixArray1, cameraMatrixArray2 };
            double[] distortionParametersArray = { 0.079800650654781, 0, 2.12199579096527E-314, 0, 1.26767782840851E-311, 1.26763894007465E-311, 0, 0, 0, 1.26739902687602E-311, 0, 0, 4.9868923567144E-316, 0 };
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

            // This code generates all the test markers. Uncomment to generate
            //for (int i = 0; i < 100; i++)
            //{
            //    Mat markerImage = new Mat();
            //    ArucoInvoke.DrawMarker(dictionary, i, 200, markerImage);
            //    markerImage.Save("Marker" + i + ".png");
            //}
        }

        /// <summary>
        /// Returns the marker locations in a new colour frame
        /// </summary>
        /// <param name="e">The colour frame data passed</param>
        /// <param name="sender">object sending the event</param>
        /// <param name="idList">a list to be populated with the marker ids placed in the return</param>
        /// <param name="rotationVectors">a list to be populated with found rotation vectors</param>
        /// <param name="translationVectors">a list to be populated with found translation vectors</param>
        /// <param name="deltaT">will be populated with the delta time since the last update (in ms)</param>
        /// <returns>The number of markers found</returns>
        public int FindMarkers(ColorImageFrameReadyEventArgs e, List<int> idList, List<double[]> rotationVectors, List<double[]> translationVectors, ref double deltaT)
        {
            if (framesProcessed++ % throttleFinding != 0) return 0;

            using (VectorOfInt ids = new VectorOfInt())
            using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
            using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame == null) return 0;

                DetectorParameters p = DetectorParameters.GetDefault();
                Bitmap frameBitmap = ImageToBitmap(frame);
                frameBitmap.RotateFlip(RotateFlipType.RotateNoneFlipX); // The Kinect sems to flip the image
                Image<Bgr, byte> imageFromKinect = new Image<Bgr, byte>(frameBitmap);

                ArucoInvoke.DetectMarkers(imageFromKinect, dictionary, corners, ids, p, rejected);

                if(ids.Size == 0) return 0;

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
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, 0.08f, cameraMatrix, distortionParameters, rvecs, tvecs);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Failed to estimate marker pose: " + ex.Message);
                    }
                    for (int i = 0; i < ids.Size; i++)
                    {
                        using (Mat rvecMat = rvecs.Row(i))
                        using (Mat tvecMat = tvecs.Row(i))
                        {
                            double[] values = new double[3];
                            rvecMat.CopyTo(values);
                            rotationVectors.Add(values);
                            tvecMat.CopyTo(values);
                            translationVectors.Add(values);
                        }
                    }
                }

                // Draw the markers
                PointF[][] outArray = corners.ToArrayOfArray();
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(System.Windows.Media.Brushes.Transparent, null, new System.Windows.Rect(0.0, 0.0, RenderWidth, RenderHeight));
                    foreach (PointF[] markerArray in outArray)
                    {
                        Mat hMat = new Mat();
                        CvInvoke.FindHomography(objCorners, markerArray, hMat, Emgu.CV.CvEnum.HomographyMethod.Default);

                        for (int i = 0; i < markerArray.Length; i++)
                        {
                            // Redo mirroring
                            markerArray[i].X = cImgWidth - markerArray[i].X;

                            // From front view, top right = blue = markerarray[0]. Rest are CCW
                            System.Windows.Point cornerPoint = new System.Windows.Point((int)markerArray[i].X/2, (int)markerArray[i].Y/2);
                            dc.DrawEllipse(
                                this.cornerBrushes[i],
                                null,
                                cornerPoint,
                                CornerThickness,
                                CornerThickness);
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
    }
}
