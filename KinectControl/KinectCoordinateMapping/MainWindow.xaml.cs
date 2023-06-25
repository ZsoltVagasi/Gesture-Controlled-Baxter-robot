namespace KinectAngleControl
{
	using Microsoft.Kinect;
	using System;
	using System.Windows;
	using NetMQ;
	using NetMQ.Sockets;
	using System.Windows.Controls;
	using System.Threading;
	using KinectCoordinateMapping;

	public partial class KinectControl : Window
    {
        KinectSensor _sensor = KinectSensor.GetDefault();
        MultiSourceFrameReader _reader;
        CameraMode _cameraMode;
		static TrackingMode _trackingMode;
		CalculationClass _calculator = new CalculationClass();
		RequestSocket _initSocket = new RequestSocket();
		static BodyFrame _bodyFrame;
		static InfraredFrame _infraredFrame;
		static ColorFrame _colorFrame;
		static DepthFrame _depthFrame;

		static Mutex mutex = new Mutex();

		Thread zmqThread = new Thread(() =>
		{
			CalculationClass calculator = new CalculationClass();
			using (var sendSocket = new RequestSocket())
			{
				KinectSensor kinectSensor = KinectSensor.GetDefault();
				BodyFrameReader bodyReader = kinectSensor.BodyFrameSource.OpenReader();
				//InfraredFrameReader infraReader = kinectSensor.InfraredFrameSource.OpenReader();
				//ColorFrameReader colorReader = kinectSensor.ColorFrameSource.OpenReader();
				//DepthFrameReader depthFrameReader = kinectSensor.DepthFrameSource.OpenReader();

				//sendSocket.Connect("tcp://localhost:5555");
				sendSocket.Connect("tcp://192.168.98.219:15558");

				while (true)
				{
					// Acquire body frame data
					BodyFrame bodyFrame = bodyReader.AcquireLatestFrame();
					//InfraredFrame infraFrame = infraReader.AcquireLatestFrame();
					//ColorFrame colorFrame = colorReader.AcquireLatestFrame();
					//DepthFrame depthFrame = depthFrameReader.AcquireLatestFrame();

					//_infraredFrame= infraFrame;
					//_bodyFrame=bodyFrame;
					//mutex.WaitOne();
					//BodyFrame bodyFrame = _bodyFrame;
					//mutex.ReleaseMutex();

					if (bodyFrame != null)
					{
						Body[] bodies = new Body[bodyFrame.BodyCount];
						bodyFrame.GetAndRefreshBodyData(bodies);

						Body body = bodies[0];

						if (body.IsTracked)
						{
							// Process body data
							Joint headJoint = body.Joints[JointType.Head];

							double angleElbowLeft = 0;
							double angleElbowRotationLeft = 0;
							double angleShoulderLeft = 0;
							double angleShoulderRotationLeft = 0;

							double angleElbowRight = 0;
							double angleShoulderRight = 0;
							double angleShoulderRotationRight = 0;
							double angleElbowRotationRight = 0;

							Joint joint = body.Joints[JointType.Neck];
							if (joint.TrackingState == TrackingState.Tracked)
							{
								CameraSpacePoint jointPosition = joint.Position;
								// Calculate Angles of left hand joints
								angleElbowLeft = calculator.CalculateAngleXYAxis(body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft], body.Joints[JointType.WristLeft]);
								angleShoulderLeft = calculator.CalculateAngleXYAxis(body.Joints[JointType.HipLeft], body.Joints[JointType.ShoulderLeft], body.Joints[JointType.WristLeft]);
								angleShoulderRotationLeft = calculator.CalculateAngleXZAxis(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft]);
								angleElbowRotationLeft = calculator.CalculateAngleXZAxisElbow(body.Joints[JointType.WristLeft], body.Joints[JointType.ElbowLeft]);
								// Calculate Angles of right hand joints
								angleElbowRight = calculator.CalculateAngleXYAxis(body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight], body.Joints[JointType.WristRight]);
								angleShoulderRight = calculator.CalculateAngleXYAxis(body.Joints[JointType.HipRight], body.Joints[JointType.ShoulderRight], body.Joints[JointType.WristRight]);
								angleShoulderRotationRight = calculator.CalculateAngleXZAxis(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight]);
								angleElbowRotationRight = calculator.CalculateAngleXZAxisElbow(body.Joints[JointType.WristRight], body.Joints[JointType.ElbowRight]);
							}

							if (body.HandLeftState == HandState.Closed)
							{
								if (_trackingMode == TrackingMode.Symmetric)
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageLeft(true));
									sendSocket.ReceiveFrameString();
								}
								else
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageRight(true));
									sendSocket.ReceiveFrameString();
								}
							}
							else
							{
								if (_trackingMode == TrackingMode.Symmetric)
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageLeft(false));
									sendSocket.ReceiveFrameString();
								}
								else
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageRight(false));
									sendSocket.ReceiveFrameString();
								}
							}
							if (body.HandRightState == HandState.Closed)
							{
								if (_trackingMode == TrackingMode.Symmetric)
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageRight(true));
									sendSocket.ReceiveFrameString();
								}
								else
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageLeft(true));
									sendSocket.ReceiveFrameString();
								}

							}
							else
							{
								if (_trackingMode == TrackingMode.Symmetric)
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageRight(false));
									sendSocket.ReceiveFrameString();
								}
								else
								{
									sendSocket.SendFrame(calculator.BiuldJsonGripperMessageLeft(false));
									sendSocket.ReceiveFrameString();
								}
							}

							// Format the calculated angles and then send it to the Gateway (both hands)
							double newAngle1Left = Math.Round(angleElbowLeft, 2);
							double newAngle2Left = Math.Round(angleShoulderLeft, 2);
							double newAngle3Left = Math.Round(angleShoulderRotationLeft, 2);
							double newAngle4Left = Math.Round(angleElbowRotationLeft, 2);
							if (body.Joints[JointType.WristLeft].Position.Y < body.Joints[JointType.ElbowLeft].Position.Y)
							{
								newAngle4Left *= -1;
							}
							double newAngle1Right = Math.Round(angleElbowRight, 2);
							double newAngle2Right = Math.Round(angleShoulderRight, 2);
							double newAngle3Right = Math.Round(angleShoulderRotationRight, 2);
							double newAngle4Right = Math.Round(angleElbowRotationRight, 2);
							if (body.Joints[JointType.WristRight].Position.Y < body.Joints[JointType.ElbowRight].Position.Y)
							{
								newAngle4Right *= -1;
							}

							//Send Left Limb angles
							if (_trackingMode == TrackingMode.Symmetric)
							{
								sendSocket.SendFrame(calculator.buildJsonAnglesLeft(newAngle1Left, newAngle4Left, newAngle2Left, newAngle3Left));
								sendSocket.ReceiveFrameString();
							}
							else
							{
								sendSocket.SendFrame(calculator.buildJsonAnglesLeft(newAngle1Right, newAngle4Right, newAngle2Right, newAngle3Right));
								sendSocket.ReceiveFrameString();
							}

							//Send right Limb angles
							if (_trackingMode == TrackingMode.Symmetric)
							{
								sendSocket.SendFrame(calculator.buildJsonAnglesRight(newAngle1Right, newAngle4Right, newAngle2Right, newAngle3Right));
								sendSocket.ReceiveFrameString();
							}
							else
							{
								sendSocket.SendFrame(calculator.buildJsonAnglesRight(newAngle1Left, newAngle4Left, newAngle2Left, newAngle3Left));
								sendSocket.ReceiveFrameString();
							}

							if (body.Joints[JointType.HandLeft].Position.X > body.Joints[JointType.HandRight].Position.X)
							{
								sendSocket.SendFrame(calculator.GoToStartLeft());
								sendSocket.ReceiveFrameString();
								sendSocket.SendFrame(calculator.GoToStartRight());
								sendSocket.ReceiveFrameString();
								return;
							}
						}
					}
				}
			}
		});

		public KinectControl()
        {
			InitializeComponent();
			var trackingModeSelected = false;
			var sensorModeSelected = false;
			var promptWindow = new Window
			{
				Title = "Select the tracking and sensor modes!",
				Width = 350,
				Height = 300,
				WindowStartupLocation = WindowStartupLocation.CenterScreen,
				Content = new StackPanel
				{
					Orientation = Orientation.Vertical,
					Margin = new Thickness(10)
				}
			};

			// Tracking mode section
			var trackingModeTextBlock = new TextBlock { Text = "Choose the tracking mode:", Margin = new Thickness(0, 0, 0, 10) };
			((StackPanel)promptWindow.Content).Children.Add(trackingModeTextBlock);

			var trackingModePanel = new StackPanel { Orientation = Orientation.Horizontal };
			var invertedButton = new Button { Content = "Symmetric", Tag = TrackingMode.Symmetric, Margin = new Thickness(0, 0, 10, 0), Height = 30, Width = 80 };
			var mirroredButton = new Button { Content = "Mirrored", Tag = TrackingMode.Mirror, Height = 30, Width = 80 };

			trackingModePanel.Children.Add(invertedButton);
			trackingModePanel.Children.Add(mirroredButton);

			((StackPanel)promptWindow.Content).Children.Add(trackingModePanel);

			// Sensor mode section
			var sensorModeTextBlock = new TextBlock { Text = "Choose the sensor mode:", Margin = new Thickness(0, 20, 0, 10) };
			((StackPanel)promptWindow.Content).Children.Add(sensorModeTextBlock);

			var sensorModePanel = new StackPanel { Orientation = Orientation.Horizontal };
			var infraredButton = new Button { Content = "Infrared", Tag = CameraMode.Infrared, Margin = new Thickness(0, 0, 10, 0), Height = 30, Width = 80 };
			var colorButton = new Button { Content = "Color", Tag = CameraMode.Color, Margin = new Thickness(0, 0, 10, 0), Height = 30, Width = 80 };
			var depthButton = new Button { Content = "Depth", Tag = CameraMode.Depth, Height = 30, Width = 80 };

			sensorModePanel.Children.Add(infraredButton);
			sensorModePanel.Children.Add(colorButton);
			sensorModePanel.Children.Add(depthButton);

			((StackPanel)promptWindow.Content).Children.Add(sensorModePanel);

			// Submit button
			var submitButton = new Button { Content = "Submit", Margin = new Thickness(0, 20, 0, 0), Height = 30, Width = 80 };
			((StackPanel)promptWindow.Content).Children.Add(submitButton);

			// Event handlers for tracking mode buttons
			invertedButton.Click += (s, e) =>
			{
				_trackingMode = TrackingMode.Symmetric;
				trackingModeSelected = true;
				invertedButton.IsEnabled = false;
				mirroredButton.IsEnabled = true;
			};

			mirroredButton.Click += (s, e) =>
			{
				_trackingMode = TrackingMode.Mirror;
				trackingModeSelected = true;
				invertedButton.IsEnabled = true;
				mirroredButton.IsEnabled = false;
			};

			// Event handlers for sensor mode buttons
			infraredButton.Click += (s, e) =>
			{
				_cameraMode = CameraMode.Infrared;
				sensorModeSelected = true;
				infraredButton.IsEnabled = false;
				colorButton.IsEnabled = true;
				depthButton.IsEnabled = true;
			};

			colorButton.Click += (s, e) =>
			{
				_cameraMode = CameraMode.Color;
				sensorModeSelected = true;
				infraredButton.IsEnabled = true;
				colorButton.IsEnabled = false;
				depthButton.IsEnabled = true;
			};

			depthButton.Click += (s, e) =>
			{
				_cameraMode = CameraMode.Depth;
				sensorModeSelected = true;
				infraredButton.IsEnabled = true;
				colorButton.IsEnabled = true;
				depthButton.IsEnabled = false;
			};

			// Event handler for Submit button
			submitButton.Click += (s, e) =>
			{
				if (trackingModeSelected && sensorModeSelected)
				{
					promptWindow.DialogResult = true;
				}
			};

			// Show the prompt window and wait for the user to choose an option or cancel
			var result = promptWindow.ShowDialog();

			if (result == true)
			{
				// Start the MainWindow
				this.Show();
			}
			else
			{
				this.Close();
			}
		}		

		// Window Load and Close event handling
		private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth  | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

				_initSocket.Connect("tcp://192.168.98.219:15558");
				//_initSocket.Connect("tcp://localhost:5555");

				_initSocket.SendFrame(_calculator.GoToStartLeft());
				_initSocket.ReceiveFrameString();

				_initSocket.SendFrame(_calculator.GoToStartRight());
				_initSocket.ReceiveFrameString();				

				_initSocket.Disconnect("tcp://192.168.98.219:15558");

				zmqThread.Start();
			}
		}

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }
            if (_sensor != null)
            {
                _sensor.Close();
            }
			if(zmqThread.IsAlive)
			{
				zmqThread.Join();
			}
        }

		// Image Drawing based on _mode (color/depth/infrared)

		void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
		{
			var reference = e.FrameReference.AcquireFrame();
			// Color
			using (var frame = reference.ColorFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					if (_cameraMode == CameraMode.Color)
					{
						camera.Source = KinectCoordinateMapping.ColorExtensions.ToBitmap(frame);
						mutex.WaitOne();
						_colorFrame = frame;
						mutex.ReleaseMutex();
					}
				}
			}

			// Depth
			using (var frame = reference.DepthFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					if (_cameraMode == CameraMode.Depth)
					{
						camera.Source = KinectCoordinateMapping.DepthExtensions.ToBitmap(frame);
						mutex.WaitOne();
						_depthFrame = frame;
						mutex.ReleaseMutex();
					}
				}
			}

			// Infrared
			using (var frame = reference.InfraredFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					if (_cameraMode == CameraMode.Infrared)
					{
						//camera.Source = KinectCoordinateMapping.InfraredExtensions.ToBitmap(frame);
						camera.Source = frame.ToBitmap();
						mutex.WaitOne();
						_infraredFrame= frame;
						mutex.ReleaseMutex();
					}
				}
			}


			using (var frame = reference.BodyFrameReference.AcquireFrame())
			{
				if (frame != null)
				{
					mutex.WaitOne();
					_bodyFrame = frame;
					mutex.ReleaseMutex();
				}
			}
		}
    }
    enum CameraMode
    {
        Color,
        Depth,
        Infrared
    }
	enum TrackingMode
	{
		Symmetric,
		Mirror
	}
}