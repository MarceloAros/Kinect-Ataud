//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{

    using System.Diagnostics;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using Matrix.Xmpp;
    using matrixXmpp = Matrix.Xmpp;
    using Matrix.Xmpp.Client;
    using System;
    using System.Timers;
    using System.Windows.Media.Media3D;

    public class Angles
    {
        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double dotProduct;
            vectorA.Normalize();
            vectorB.Normalize();
            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return (double)Math.Acos(dotProduct) / Math.PI * 180;
        }

        public byte[] GetVector(Skeleton skeleton)
        {
            Vector3D ShoulderCenter = new Vector3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
            Vector3D RightShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D LeftShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D RightElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D LeftElbow = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D RightWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            Vector3D LeftWrist = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
            Vector3D UpVector = new Vector3D(0.0, 1.0, 0.0);

            double AngleRightElbow = AngleBetweenTwoVectors(RightElbow - RightShoulder, RightElbow - RightWrist);
            double AngleRightShoulder = AngleBetweenTwoVectors(UpVector, RightShoulder - RightElbow);
            double AngleLeftElbow = AngleBetweenTwoVectors(LeftElbow - LeftShoulder, LeftElbow - LeftWrist);
            double AngleLeftShoulder = AngleBetweenTwoVectors(UpVector, LeftShoulder - LeftElbow);


            byte[] Angles = { Convert.ToByte(AngleRightElbow), Convert.ToByte(AngleRightShoulder), Convert.ToByte(AngleLeftElbow), Convert.ToByte(AngleLeftShoulder) };
            return Angles;
        }
    }

    public partial class MainWindow : Window
    {

        private const float RenderWidth = 640.0f;

        /// Height of our output drawing
        private const float RenderHeight = 480.0f;

        /// Thickness of drawn joint lines
        private const double JointThickness = 3;

        /// Thickness of body center ellipse
        private const double BodyCenterThickness = 10;

        /// Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;

        /// Brush used to draw skeleton center point
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// Brush used for drawing joints that are currently inferred     
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// Pen used for drawing bones that are currently tracked
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// Pen used for drawing bones that are currently inferred   
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// Active Kinect sensor
        private KinectSensor sensor;

        /// Drawing group for skeleton rendering output
        private DrawingGroup drawingGroup;

        /// Drawing image that we will display
        private DrawingImage imageSource;

        /// Library to conect XMPP server
        XmppClient xmppClient;

        /// Counter for help auxiliar
        private int contador = 0;

        // Timer para detectar giro
        System.Timers.Timer timer = new System.Timers.Timer();

        // Timer para detectar persona frente a la Kinect
        System.Timers.Timer timerDectecionPersona = new System.Timers.Timer();

        //Identificador de persona frente a la kinect
        int idTracking = -100;

        /// Contador auxiliar si se paso por ciertos angulos
        private bool[] aux = new bool[4];

        /// Variable que detecta el giro
        bool detectTurn = false;

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

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            xmppClient = new XmppClient{
                // Ingresar credenciales de acceso aquí
                XmppDomain = "xmpp.binarylamp.cl",
                Username = "totem",
                Password = "totem",
                Resource = "test"
            };

            xmppClient.OnMessage += XmppClient_OnMessage;

            xmppClient.Open();

            timer.Enabled = true;
            timer.Interval = 2000;
            timer.Elapsed += new System.Timers.ElapsedEventHandler(timerElapsedTime);



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
            foreach (var potentialSensor in KinectSensor.KinectSensors) {
                if (potentialSensor.Status == KinectStatus.Connected) {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor) {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try {
                    this.sensor.Start();
                } catch (IOException) {
                    this.sensor = null;
                }
            }

            if (null == this.sensor) {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        private void timerElapsedTime(object sender, ElapsedEventArgs e) {
            //Debug.WriteLine("Paso un segundo. Detect turn: " + detectTurn);
            detectTurn = (aux[0] && aux[1] && aux[2] && aux[3]) ? true : false;
            if (detectTurn) { Debug.WriteLine("Giro Detectado"); detectTurn = false; }
            for (int i=0; i<aux.Length; i++) {
                aux[i] = false;
            }
        }

        private void XmppClient_OnMessage(object sender, MessageEventArgs e)
        {
            if (e.Message.Body != null) {
                Debug.WriteLine(string.Format(">In : {2}\n\tFrom: {0}\n\tBody: {1}", e.Message.From, e.Message.Body, e.Message.Type));

                if (e.Message.Body == "kinect") {
                    String envioRespuesta = (detectTurn) ? "true" : "false;";
                    Message mensajeAEnviar = new Message {
                        To = e.Message.From,
                        Type = MessageType.Chat,
                        Body = "kinect:" + envioRespuesta
                    };
                    Debug.WriteLine(string.Format(">Out: {2}\n\tFrom: {0}\n\tBody: {1}", mensajeAEnviar.From, mensajeAEnviar.Body, mensajeAEnviar.Type));

                    xmppClient.Send(mensajeAEnviar);
                } else {
                    xmppClient.Send(new Message {
                        To = e.Message.From,
                        Type = MessageType.Chat,
                        Body = "ERROR: Soy un bot tonto. No entiendo la peticion. Intenta escribiendo y mandando \"kinect\"",
                        XHtml = new matrixXmpp::XHtmlIM.Html
                        {
                            Body = new matrixXmpp.XHtmlIM.Body  {
                                InnerXHtml = "<p><i>ERROR:</i> <strong>No entiendo la peticion:</strong> Intenta: \"kinect\"</p>"
                            }
                        }
                    });
                }
            }
        }

        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e) {
            xmppClient.Close();
            if (null != this.sensor) {
                this.sensor.Stop();
            }
        }

        
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e) { 
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())  {
                if (skeletonFrame != null) {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open()) {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0) {
                    contador += 1;
                    if (contador > 59) {
                        contador = 0;
                    }
                    foreach (Skeleton skel in skeletons)  {
                        RenderClippedEdges(skel, dc);
    
                        if (skel.TrackingState == SkeletonTrackingState.Tracked) {

                            Angles MyAngles = new Angles();
                            byte[] ReadyAngles = MyAngles.GetVector(skel);
                            statusBarText.Text = "CodoDerecho: " + ReadyAngles[0].ToString() + "\t HombroDerecho: " + ReadyAngles[1].ToString() + "\t CodoIzq: " + ReadyAngles[2].ToString() + "\t HombroIzq: " + ReadyAngles[3].ToString();


                            //######### Inicio: Fill Aux Angles ###########
                            if (ReadyAngles[0] < 40) { aux[0] = true; }
                            if (ReadyAngles[0] >= 40 && ReadyAngles[0] < 80) { aux[1] = true; }
                            if (ReadyAngles[0] >= 80 && ReadyAngles[0] < 120) { aux[2] = true; }
                            if (ReadyAngles[0] >= 120 && ReadyAngles[0] < 160) { aux[3] = true; }

                            statusBarText.Text += "\t" + aux[0] + " " + aux[1] + " " + aux[2] + " " + aux[3];

                            //######### Fin: Fill Aux Angles ###########
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly) {
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
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext) {
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
            foreach (Joint joint in skeleton.Joints) {
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
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint) {
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
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1) {
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
                joint1.TrackingState == JointTrackingState.Inferred) {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked) {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e) {
            if (null != this.sensor) {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault()) {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

    }
}