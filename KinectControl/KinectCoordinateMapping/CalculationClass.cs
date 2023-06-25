using LightBuzz.Vitruvius;
using Microsoft.Kinect;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectCoordinateMapping
{
	public class CalculationClass
	{
		public double CalculateAngleXYAxis(Joint joint1, Joint joint2, Joint joint3)
		{			
			return joint2.Angle(joint1, joint3);
		}

		public double CalculateAngleXZAxis(Joint joint1, Joint joint2, Joint joint3)
		{
			CameraSpacePoint position1 = joint1.Position;
			CameraSpacePoint position2 = joint2.Position;
			CameraSpacePoint position3 = joint3.Position;

			double[] vector1 = new double[] { position2.X - position1.X, 0, position2.Z - position1.Z };
			double[] vector2 = new double[] { position3.X - position2.X, 0, position3.Z - position2.Z };

			double dotProduct = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
			double magnitude1 = Math.Sqrt(vector1[0] * vector1[0] + vector1[1] * vector1[1] + vector1[2] * vector1[2]);
			double magnitude2 = Math.Sqrt(vector2[0] * vector2[0] + vector2[1] * vector2[1] + vector2[2] * vector2[2]);

			double angleInRadians = Math.Acos(dotProduct / (magnitude1 * magnitude2));
			double angleInDegrees = angleInRadians * 180.0 / Math.PI;

			return angleInDegrees;
		}

		public double CalculateAngleXZAxisElbow(Joint joint1, Joint joint2)
		{
			CameraSpacePoint position1 = joint1.Position;
			CameraSpacePoint position2 = joint2.Position;
			CameraSpacePoint position3 = joint2.Position;
			position3.Z = position3.Z + 80;

			double[] vector1 = new double[] { position2.X - position1.X, 0, position2.Z - position1.Z };
			double[] vector2 = new double[] { position3.X - position2.X, 0, position3.Z - position2.Z };

			double dotProduct = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
			double magnitude1 = Math.Sqrt(vector1[0] * vector1[0] + vector1[1] * vector1[1] + vector1[2] * vector1[2]);
			double magnitude2 = Math.Sqrt(vector2[0] * vector2[0] + vector2[1] * vector2[1] + vector2[2] * vector2[2]);

			double angleInRadians = Math.Acos(dotProduct / (magnitude1 * magnitude2));
			double angleInDegrees = angleInRadians * 180.0 / Math.PI;

			return angleInDegrees;
		}

		public string GoToStartLeft()
		{
			var cmd = new Dictionary<string, Dictionary<string, object>>
			{
				{
					"left_arm", new Dictionary<string, object>
					{
						{ "move_to",  new Dictionary<string, object>
							{
								{"left_w0", -0.044},
								{"left_w1", 0.017},
								{"left_w2", -0.059},
								{"left_e0", 0.046},
								{"left_e1", 2.054},
								{"left_s0", 0.711},
								{"left_s1", -0.572}
							}
						}
					}
				}
			};
			string toSend = JsonConvert.SerializeObject(cmd);
			return toSend;
		}

	public string GoToStartRight()
	{
		var cmd = new Dictionary<string, Dictionary<string, object>>
		{
			{
				"right_arm", new Dictionary<string, object>
				{
					{ "move_to",  new Dictionary<string, object>
						{
							{"right_w0", 0.172},
							{"right_w1", -0.039},
							{"right_w2", -0.156},
							{"right_e0", -0.069},
							{"right_e1", 2.172},
							{"right_s0", -0.682},
							{"right_s1", -0.643}
						}
					}
				}
			}
		};
		return JsonConvert.SerializeObject(cmd);
	}

		public string buildJsonAnglesLeft(double angleElbow, double angleElbowRotation, double angleShoulder, double angleShoulderRotation)
		{
			double angle1 = angleElbow * (-0.016) + 2.892;		
			double angle2 = angleShoulder * (-0.016) + 1.254;
			double angle3 = angleShoulderRotation * (-0.018) + 0.85;
			double angle4 = angleElbowRotation * (-0.014) - 1.42;

			if (angle1 > 2.48)
				angle1 = 2.48;
			if (angle1 < 0.01)
				angle1 = 0.01;
			if (angle2 > 1.01)
				angle2 = 1.01;
			if (angle2 < -1.55)
				angle2 = -1.55;
			if (angle3 > 0.85)
				angle3 = 0.85;
			if (angle3 < -0.82)
				angle3 = -0.82;
			if (angle4 < -2.69)
				angle4 = -2.69;
			if (angle4 > -0.15)
				angle4 = -0.15;

			var cmd = new Dictionary<string, Dictionary<string, object>>
			{
				{
					"left_arm", new Dictionary<string, object>
					{
						{ "set_joint_positions",  new Dictionary<string, object>
							{
								{"left_w0", 0.579},
								{"left_w1", 0.934},
								{"left_w2", 0.001},
								{"left_e0", angle4},
								{"left_e1", angle1},
								{"left_s0", angle3},
								{"left_s1", angle2}
							}
						}
					}
				}
			};
			string toSend = JsonConvert.SerializeObject(cmd);
			return toSend;
		}

		public string buildJsonAnglesRight(double angleElbow, double angleElbowRotation, double angleShoulder, double angleShoulderRotation)
		{
			double angle2 = angleElbow * (-0.016) + 2.891;
			double angle3 = angleShoulder * (-0.016) + 1.254;
			double angle4 = angleShoulderRotation * (-0.018) + 0.85;
			double angle5 = angleElbowRotation * (-0.014) - 1.42;
			if (angle2 > 2.48)
				angle2 = 2.48;
			if (angle3 > 1.01)
				angle3 = 1.01;
			if (angle3 < -1.55)
				angle3 = -1.55;
			if (angle4 > 0.85)
				angle4 = 0.85;
			if (angle4 < -0.82)
				angle4 = -0.82;
			if (angle5 < -2.69)
				angle5 = -2.69;
			if (angle5 > -0.15)
				angle5 = -0.15;
			var cmd = new Dictionary<string, Dictionary<string, object>>
			{
				{
					"right_arm", new Dictionary<string, object>
					{
						{ "set_joint_positions",  new Dictionary<string, object>
							{
								{"right_w0", 0.3459126676681608},
								{"right_w1", 1.0734030563228183},
								{"right_w2", 2.8294275632546455},
								{"right_e0", angle5*(-1)},
								{"right_e1", angle2},
								{"right_s0", angle4*(-1)},
								{"right_s1", angle3}
							}
						}
					}
				}
			};
			string toSend = JsonConvert.SerializeObject(cmd);
			return toSend;
		}

		public string BiuldJsonGripperMessageLeft(bool state)
		{
			string toSend = "";
			if (state == true)
			{
				var cmd = new Dictionary<string, Dictionary<string, object>>
				{
					{
						"left_arm", new Dictionary<string, object>
						{
							{ "gripper", 0 }
						}
					}
				};
				toSend = JsonConvert.SerializeObject(cmd);
			}
			else
			{
				var cmd = new Dictionary<string, Dictionary<string, object>>
				{
					{
						"left_arm", new Dictionary<string, object>
						{
							{ "gripper", 1 }
						}
					}
				};
				toSend = JsonConvert.SerializeObject(cmd);
			}
			return toSend;
		}

		public string BiuldJsonGripperMessageRight(bool state)
		{
			string toSend = "";
			if (state == true)
			{
				var cmd = new Dictionary<string, Dictionary<string, object>>
				{
					{
						"right_arm", new Dictionary<string, object>
						{
							{ "gripper", 0 }
						}
					}
				};
				toSend = JsonConvert.SerializeObject(cmd);
			}
			else
			{
				var cmd = new Dictionary<string, Dictionary<string, object>>
				{
					{
						"right_arm", new Dictionary<string, object>
						{
							{ "gripper", 1 }
						}
					}
				};
				toSend = JsonConvert.SerializeObject(cmd);
			}
			return toSend;
		}
	}
}

//angleElbowLeft = body.Joints[JointType.ElbowLeft].Angle(body.Joints[JointType.ShoulderLeft], body.Joints[JointType.WristLeft]);
//angleShoulderLeft = body.Joints[JointType.ShoulderLeft].Angle(body.Joints[JointType.HipLeft], body.Joints[JointType.ElbowLeft]);
//angleWristLeft = body.Joints[JointType.HandLeft].Angle(body.Joints[JointType.HandTipLeft], body.Joints[JointType.ElbowLeft]);

//angleElbowRight = body.Joints[JointType.ElbowRight].Angle(body.Joints[JointType.ShoulderRight], body.Joints[JointType.WristRight]);
//angleShoulderRight = body.Joints[JointType.ShoulderRight].Angle(body.Joints[JointType.HipRight], body.Joints[JointType.ElbowRight]);
//angleWristRight = body.Joints[JointType.HandRight].Angle(body.Joints[JointType.HandTipRight], body.Joints[JointType.ElbowRight]);

