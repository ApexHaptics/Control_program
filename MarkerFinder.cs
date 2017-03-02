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

        /// <summary>
        /// Constructor for the MarkerFinder class
        /// </summary>
        /// <param name="drawingGroup">Where the skeleton will be drawn onto</param>
        public MarkerFinder(DrawingGroup drawingGroup)
        {
            this.drawingGroup = drawingGroup;

            // This code generates all the test markers. Uncomment to generate
            //for (int i = 0; i < 100; i++)
            //{
            //    Mat markerImage = new Mat();
            //    ArucoInvoke.DrawMarker(dictionary, i, 200, markerImage);
            //    markerImage.Save("Marker" + i + ".png");
            //}
        }

        /// <summary>
        /// A method to handle a new color frame. Returns marker locations
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="idList">a list to be populated with the marker ids placed in the return</param>
        /// <returns>The array of points detected in the image</returns>
        public PointF[][] FindMarkers(ColorImageFrameReadyEventArgs e, List<int> idList)
        {
            if (framesProcessed++ % throttleFinding != 0) return null;

            using (VectorOfInt ids = new VectorOfInt())
            using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
            using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame == null) return null;

                DetectorParameters p = DetectorParameters.GetDefault();
                Bitmap frameBitmap = ImageToBitmap(frame);
                frameBitmap.RotateFlip(RotateFlipType.RotateNoneFlipX); // The Kinect sems to flip the image
                Image<Bgr, byte> imageFromKinect = new Image<Bgr, byte>(frameBitmap);

                ArucoInvoke.DetectMarkers(imageFromKinect, dictionary, corners, ids, p, rejected);

                // Populate the marker ids
                for (int i = 0; i < ids.Size; i++)
                {
                    idList.Add(ids[i]);
                }
                
                // Draw the markers
                PointF[][] outArray = corners.ToArrayOfArray();
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(System.Windows.Media.Brushes.Transparent, null, new System.Windows.Rect(0.0, 0.0, RenderWidth, RenderHeight));
                    foreach (PointF[] markerArray in outArray)
                    {
                        for(int i = 0; i < markerArray.Length; i++)
                        {
                            // Undo mirroring
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
