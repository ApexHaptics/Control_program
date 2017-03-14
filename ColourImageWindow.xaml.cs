using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    /// <summary>
    /// Interaction logic for Window1.xaml
    /// </summary>
    public partial class ColourImageWindow : Window
    {
        /// <summary>
        /// The drawing group for the main image
        /// </summary>
        public DrawingGroup markerDrawingGroup = new DrawingGroup();

        /// <summary>
        /// The thresholding value for marker thresholding
        /// </summary>
        public int thresholdValue = 127;

        /// <summary>
        /// Whether we should be calibrating the marker
        /// </summary>
        public bool shouldCalibrate = false;

        /// <summary>
        /// Standard constructor
        /// </summary>
        public ColourImageWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="bitmap"></param>
        public void setImageBitmapSource(Bitmap bitmap)
        {
            Dispatcher.BeginInvoke(new Action(() => {
                if (!(bool)DisplayCheckBox.IsChecked) return;
                Image.Source = BitmapToImageSource(bitmap);
            }));
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            ImageSource markerImageSource = new DrawingImage(markerDrawingGroup);
            MarkerImage.Source = markerImageSource;
        }

        /// <summary>
        /// Method to convert Bitmap to ImageSource
        /// http://stackoverflow.com/questions/22499407/how-to-display-a-bitmap-in-a-wpf-image
        /// </summary>
        /// <param name="bitmap">Input Bitmap</param>
        /// <returns>Output ImageSource</returns>
        BitmapImage BitmapToImageSource(Bitmap bitmap)
        {
            using (MemoryStream memory = new MemoryStream())
            {
                bitmap.Save(memory, System.Drawing.Imaging.ImageFormat.Bmp);
                memory.Position = 0;
                BitmapImage bitmapimage = new BitmapImage();
                bitmapimage.BeginInit();
                bitmapimage.StreamSource = memory;
                bitmapimage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapimage.EndInit();

                return bitmapimage;
            }
        }

        private void ThresholdBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Return)
            {
                if (string.IsNullOrEmpty(ThresholdBox.Text)) return;
                thresholdValue = int.Parse(ThresholdBox.Text);
                ThresholdBox.Text = "";
            }
        }

        private void CalibButton_Click(object sender, RoutedEventArgs e)
        {
            shouldCalibrate = !shouldCalibrate;
            if(shouldCalibrate)
            {
                CalibButton.Content = "Stop Calib";
                CalibButton.Background = System.Windows.Media.Brushes.DarkRed;
            }
            else
            {
                CalibButton.Content = "Begin Calib";
                CalibButton.Background = System.Windows.Media.Brushes.DarkGreen;
            }
        }

        private void DisplayCheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            Image.Source = BitmapSource.Create(
                2,
                2,
                96,
                96,
                PixelFormats.Indexed1,
                new BitmapPalette(new List<System.Windows.Media.Color> { Colors.Transparent }),
                new byte[] { 0, 0, 0, 0 },
                1);
        }
    }
}
