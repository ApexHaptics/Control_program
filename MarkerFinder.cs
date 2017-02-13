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
        private const int cImgWidth = 640;

        /// <summary>
        /// Height of colour image
        /// </summary>
        private const int cImgHeight = 480;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Translates an image point to XYZ through the Kinect API
        /// </summary>
        private CoordinateMapper coordinateMapper;

        /// <summary>
        /// The dictionary used to detect markers
        /// </summary>
        private readonly Dictionary dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict5X5_100);

        /// <summary>
        /// The depthPixels to be assigned by coordinateMapper
        /// </summary>
        private DepthImagePixel[] depthPixels = new DepthImagePixel[cImgWidth * cImgHeight];

        /// <summary>
        /// The depthPoints to be assigned by coordinateMapper
        /// </summary>
        private DepthImagePoint[] depthPoints = new DepthImagePoint[cImgWidth * cImgHeight];

        /// <summary>
        /// Brush used to draw marker center point
        /// </summary>
        private readonly System.Windows.Media.Brush centerPointBrush = System.Windows.Media.Brushes.Blue;

        /// <summary>
        /// Thickness of marker center ellipse
        /// </summary>
        private const double MarkerCenterThickness = 10;

        /// <summary>
        /// Constructor for the SkeletonRender class
        /// </summary>
        /// <param name="drawingGroup">Where the skeleton will be drawn onto</param>
        /// <param name="coordinateMapper">The CoordinateMapper to translate an image point to XYZ</param>
        public MarkerFinder(DrawingGroup drawingGroup, CoordinateMapper coordinateMapper)
        {
            this.drawingGroup = drawingGroup;
            this.coordinateMapper = coordinateMapper;

            // This code generates a test marker. Uncomment to generate a new one
            //Mat markerImage = new Mat();
            //int markerId = 50;
            //ArucoInvoke.DrawMarker(dictionary, markerId, 200, markerImage);
            //markerImage.Save("Test.png");
        }

        /// <summary>
        /// A method to handle a new color frame. Returns marker locations
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <returns>The array of points detected in the image</returns>
        public PointF[][] FindMarkers(ColorImageFrameReadyEventArgs e)
        {
            using (VectorOfInt ids = new VectorOfInt())
            using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
            using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame == null) return null;
                coordinateMapper.MapColorFrameToDepthFrame(frame.Format, DepthImageFormat.Resolution640x480Fps30, depthPixels, depthPoints);

                DetectorParameters p = DetectorParameters.GetDefault();
                Bitmap frameBitmap = ImageToBitmap(frame);
                frameBitmap.RotateFlip(RotateFlipType.RotateNoneFlipX); // The Kinect sems to flip the image
                Image<Bgr, byte> imageFromKinect = new Image<Bgr, byte>(frameBitmap);

                ArucoInvoke.DetectMarkers(imageFromKinect, dictionary, corners, ids, p, rejected);
                PointF[][] outArray = corners.ToArrayOfArray();

                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(System.Windows.Media.Brushes.Transparent, null, new System.Windows.Rect(0.0, 0.0, RenderWidth, RenderHeight));
                    foreach (PointF[] markerArray in outArray)
                    {
                        // Find the center
                        PointF center = new PointF((markerArray[0].X + markerArray[1].X + markerArray[2].X + markerArray[3].X)/4,
                            (markerArray[0].Y + markerArray[1].Y + markerArray[2].Y + markerArray[3].Y) / 4);

                        // Convert to depth
                        int colorX = cImgWidth - (int)center.X; // Undo mirroring
                        int colorY = (int)center.Y;

                        DepthImagePoint depthPoint = depthPoints[cImgWidth * colorY + colorX];
                        // For some reason this doesn't work. Both depth->color and color->depth mappings return 0 as point locations...

                        dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            new System.Windows.Point(colorX, colorY),
                            MarkerCenterThickness,
                            MarkerCenterThickness);
                    }
                }

                return outArray;
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
