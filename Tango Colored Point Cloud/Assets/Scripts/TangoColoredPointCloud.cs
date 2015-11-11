using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Tango;

/// <summary>
/// Point cloud visualize using depth frame API.
/// </summary>
public class TangoColoredPointCloud : MonoBehaviour, ITangoDepth
{
	/// <summary>
	/// Render Camera to retrieve Video Overlay RGB Texture
	/// </summary>
	public Camera ARcam;
	private int rgb_width;
	private int rgb_height; 

	/// <summary>
	/// If set, the point cloud's mesh gets updated (much slower, useful for debugging).
	/// </summary>
	public bool m_updatePointsMesh;

	/// <summary>
	/// If set, m_updatePointsMesh also gets set at start. Then PointCloud material's renderqueue is set to background
	/// (which is same as YUV2RGB Shader) so that PointCloud data gets written to Z buffer for Depth test with virtual
	/// objects in scene. Note this is a very rudimentary way of doing occlusion and limited by the capabilities of
	/// depth camera.
	/// </summary>
	public bool m_enableOcclusion;
	
	/// <summary>
	/// The points of the point cloud, in world space.
	/// 
	/// Note that not every member of this array will be filled out, see m_pointsCount.
	/// </summary>
	private Vector3[] m_points;
	private Color[] m_colors;
	
	/// <summary>
	/// The number of points in m_points.
	/// </summary>
	private int m_pointsCount = 0;
	
	/// <summary>
	/// Time between the last two depth events.
	/// </summary>
	private float m_depthDeltaTime = 0.0f;
	
	/// <summary>
	/// The maximum points displayed.  Just some const value.
	/// </summary>
	private const int MAX_POINT_COUNT = 61440;
	
	/// <summary>
	/// The Background renderqueue's number.
	/// </summary>
	private const int BACKGROUND_RENDER_QUEUE = 1000;
	
	/// <summary>
	/// Point size of PointCloud data when projected on to image plane.
	/// </summary>
	private const int POINTCLOUD_SPLATTER_UPSAMPLE_SIZE = 30;
	
	private TangoApplication m_tangoApplication;
	
	// Matrices for transforming pointcloud to world coordinates.
	// This equation will take account of the camera sensors extrinsic.
	// Full equation is:
	// Matrix4x4 unityWorldTDepthCamera = 
	// m_unityWorldTStartService * m_startServiceTDevice * Matrix4x4.Inverse(m_imuTDevice) * m_imuTDepthCamera;
	private Matrix4x4 m_unityWorldTStartService = new Matrix4x4();
	private Matrix4x4 m_startServiceTDevice = new Matrix4x4();
	private Matrix4x4 m_imuTDevice = new Matrix4x4();
	private Matrix4x4 m_imuTDepthCamera = new Matrix4x4();
	private Matrix4x4 m_imuTColorCamera = new Matrix4x4();
	private Matrix4x4 m_ccTDepth = new Matrix4x4();
	
	/// <summary>
	/// Mesh this script will modify.
	/// </summary>
	private Mesh m_mesh;
	
	// Logging data.
	private double m_previousDepthDeltaTime = 0.0;
	private bool m_isExtrinsicQuerable = false;
	
	private Renderer m_renderer;

	private int countFrame;
	private int depthFrameRate;
	
	private TangoCameraIntrinsics intrinsics;
	
	/// <summary>
	/// Use this for initialization.
	/// </summary>
	public void Start() 
	{
		m_tangoApplication = FindObjectOfType<TangoApplication>();
		m_tangoApplication.Register(this);
		m_tangoApplication.RegisterOnTangoConnect(_SetUpExtrinsics);
		m_tangoApplication.RegisterOnTangoConnect(_SetCameraIntrinsics);
		
		m_unityWorldTStartService.SetColumn(0, new Vector4(1.0f, 0.0f, 0.0f, 0.0f));
		m_unityWorldTStartService.SetColumn(1, new Vector4(0.0f, 0.0f, 1.0f, 0.0f));
		m_unityWorldTStartService.SetColumn(2, new Vector4(0.0f, 1.0f, 0.0f, 0.0f));
		m_unityWorldTStartService.SetColumn(3, new Vector4(0.0f, 0.0f, 0.0f, 1.0f));
		
		// Assign triangles, note: this is just for visualizing point in the mesh data.
		m_points = new Vector3[MAX_POINT_COUNT];
		m_colors = new Color[MAX_POINT_COUNT];
		m_mesh = GetComponent<MeshFilter>().mesh;
		m_mesh.Clear();
		
		m_renderer = GetComponent<Renderer>();

		if (m_enableOcclusion) 
		{        
			// Set the renderpass as background renderqueue's number minus one. YUV2RGB shader executes in 
			// Background queue which is 1000.
			// But since we want to write depth data to Z buffer before YUV2RGB shader executes so that YUV2RGB 
			// data ignores Ztest from the depth data we set renderqueue of PointCloud as 99
			m_renderer.enabled = true;
			m_renderer.material.renderQueue = BACKGROUND_RENDER_QUEUE - 1;
			m_renderer.material.SetFloat("point_size", POINTCLOUD_SPLATTER_UPSAMPLE_SIZE);
			m_updatePointsMesh = true;
		}
		
		intrinsics = new TangoCameraIntrinsics();
		// Texture which will contain rgb colors
		//tex = new Texture2D(rgb_width, rgb_height, TextureFormat.RGB24 , false);
		
		depthFrameRate = 1;
		countFrame = 0;

		rgb_width = ARcam.targetTexture.width;
		rgb_height = ARcam.targetTexture.height;
	}
	
	/// <summary>
	/// Callback that gets called when depth is available from the Tango Service.
	/// </summary>
	/// <param name="tangoDepth">Depth information from Tango.</param>
	public void OnTangoDepthAvailable(TangoUnityDepth tangoDepth)
	{
		
		// Get RGB Video
		/*RenderTexture.active = rt; 
		*/
		/*tex.ReadPixels(new Rect(0,0,rgb_width, rgb_height), 0, 0);
		RenderTexture.active = null; // added to avoid errors 
		Color[] pix = tex.GetPixels();
*/
		// Calculate the time since the last successful depth data
		// collection.
		if (m_previousDepthDeltaTime == 0.0)
		{
			m_previousDepthDeltaTime = tangoDepth.m_timestamp;
		}
		else
		{
			m_depthDeltaTime = (float)((tangoDepth.m_timestamp - m_previousDepthDeltaTime) * 1000.0);
			m_previousDepthDeltaTime = tangoDepth.m_timestamp;
		}
		
		if((countFrame%depthFrameRate) != 0)
		{
			countFrame += 1;
			return;
		}
		countFrame += 1;
		
		// Fill in the data to draw the point cloud.
		if (tangoDepth != null && tangoDepth.m_points != null)
		{
			m_pointsCount = tangoDepth.m_pointCount;
			if (m_pointsCount > 0)
			{
				TangoCoordinateFramePair pair;
				TangoPoseData poseData = new TangoPoseData();
				
				// Query pose to transform point cloud to world coordinates, here we are using the timestamp
				// that we get from depth.
				pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_START_OF_SERVICE;
				pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
				PoseProvider.GetPoseAtTime(poseData, m_previousDepthDeltaTime, pair);
				if (poseData.status_code != TangoEnums.TangoPoseStatusType.TANGO_POSE_VALID)
				{
					return;
				}
				
				Color[] pix = RTImage();

				Vector3 position = new Vector3((float)poseData.translation[0],
				                               (float)poseData.translation[1],
				                               (float)poseData.translation[2]);
				Quaternion quat = new Quaternion((float)poseData.orientation[0],
				                                 (float)poseData.orientation[1],
				                                 (float)poseData.orientation[2],
				                                 (float)poseData.orientation[3]);
				m_startServiceTDevice = Matrix4x4.TRS(position, quat, Vector3.one);
				
				// The transformation matrix that represents the pointcloud's pose. 
				// Explanation: 
				// The pointcloud which is in Depth camera's frame, is put in unity world's 
				// coordinate system(wrt unity world).
				// Then we are extracting the position and rotation from uwTuc matrix and applying it to 
				// the PointCloud's transform.
				Matrix4x4 unityWorldTDepthCamera = m_unityWorldTStartService * m_startServiceTDevice * Matrix4x4.Inverse(m_imuTDevice) * m_imuTDepthCamera;
				transform.position = Vector3.zero;
				transform.rotation = Quaternion.identity;

				// Converting depth point "depth world" to "unity world"
				// and get the corresponding color
				for (int i = 0; i < m_pointsCount; ++i)
				{
					float x = tangoDepth.m_points[(i * 3) + 0];
					float y = tangoDepth.m_points[(i * 3) + 1];
					float z = tangoDepth.m_points[(i * 3) + 2];
					
					Vector3 currentPoint = new Vector3(x,y,z);
					m_points[i] = unityWorldTDepthCamera.MultiplyPoint(currentPoint);
					m_colors[i] = retrieveColorDepthPoint(pix, currentPoint);
				}
				
				if (m_updatePointsMesh)
				{
					// Need to update indicies too!
					int[] indices = new int[m_pointsCount];
					for (int i = 0; i < m_pointsCount; ++i)
					{
						indices[i] = i;
					}
					
					m_mesh.Clear();
					m_mesh.vertices = m_points;
					m_mesh.colors = m_colors;
					m_mesh.SetIndices(indices, MeshTopology.Points, 0);
				}
			}
			
		}
	}

	// Read the ARcam Renderer and return the RGB video overlay texture
	Color[] RTImage()
	{
		RenderTexture.active = ARcam.targetTexture;
		Texture2D image = new Texture2D(rgb_width, rgb_height);
		image.ReadPixels(new Rect(0, 0, rgb_width, rgb_height), 0, 0);
		image.Apply();
		return image.GetPixels();
	}

	// Project Depth point to Color point
	Vector3 projectDepthToColor(Vector3 currentPoint)
	{
		float px, py, pz;
		
		float fx = (float)intrinsics.fx;
		float fy = (float)intrinsics.fy;
		float cx = (float)intrinsics.cx;
		float cy = (float)intrinsics.cy;
		
		float
			r00 = m_ccTDepth.m00,
			r01 = m_ccTDepth.m01,
			r02 = m_ccTDepth.m02,
			tx = m_ccTDepth.m03, 
			r10 = m_ccTDepth.m10,
			r11 = m_ccTDepth.m11,
			r12 = m_ccTDepth.m12,
			ty = m_ccTDepth.m13,
			r20 = m_ccTDepth.m20,
			r21 = m_ccTDepth.m21,
			r22 = m_ccTDepth.m22,
			tz = m_ccTDepth.m23;
		
		float a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23;
		
		// KP
		a00 = fx * r00 + cx * r20;
		a01 = fx * r01 + cx * r21;
		a02 = fx * r02 + cx * r22;
		a03 = fx * tx  + cx * tz;
		
		a10 = fy * r10 + cy * r20;
		a11 = fy * r11 + cy * r21;
		a12 = fy * r12 + cy * r22;
		a13 = fy * ty  + cy * tz;
		
		a20 = r20;
		a21 = r21;
		a22 = r22;
		a23 = tz;
		
		// KP * X
		px = (currentPoint.x) * a00 + (currentPoint.y) * a01 + (currentPoint.z) * a02 + a03;
		py = (currentPoint.x) * a10 + (currentPoint.y) * a11 + (currentPoint.z) * a12 + a13;
		pz = (currentPoint.x) * a20 + (currentPoint.y) * a21 + (currentPoint.z) * a22 + a23;
		
		return new Vector3(px, py, pz);
	}

	// Project depth point on the rgb overlay texture and return the corresponding color
	private Color retrieveColorDepthPoint(Color[] rgbOverlay, Vector3 depthPoint)
	{
		Vector3 reprojectedP = projectDepthToColor (depthPoint);
		int projectX; 
		int projectY; 
		
		if (reprojectedP.z != 0.0f) {
			projectX = (int)(reprojectedP.x / reprojectedP.z);
	 		projectY = (int)(reprojectedP.y / reprojectedP.z);
		} else {
			projectX = (int)reprojectedP.x;
			projectY = (int)reprojectedP.y;
		}
		
		int index = (rgb_height - projectY) * rgb_width + projectX;
		if ((index < rgb_width * rgb_height) && (index > 0)) {
			return rgbOverlay [index];
		} else {
			return new Color (0.0f, 0.0f, 0.0f);
		}
	}


	
	private void _SetUpExtrinsics()
	{
		double timestamp = 0.0;
		TangoCoordinateFramePair pair;
		TangoPoseData poseData = new TangoPoseData();
		
		// Query the extrinsics between IMU and device frame.
		pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
		pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
		PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
		Vector3 position = new Vector3((float)poseData.translation[0],
		                               (float)poseData.translation[1],
		                               (float)poseData.translation[2]);
		Quaternion quat = new Quaternion((float)poseData.orientation[0],
		                                 (float)poseData.orientation[1],
		                                 (float)poseData.orientation[2],
		                                 (float)poseData.orientation[3]);
		m_imuTDevice = Matrix4x4.TRS(position, quat, new Vector3(1.0f, 1.0f, 1.0f));
		
		// Query the extrinsics between IMU and color camera frame.
		pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
		pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
		PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
		position = new Vector3((float)poseData.translation[0],
		                       (float)poseData.translation[1],
		                       (float)poseData.translation[2]);
		quat = new Quaternion((float)poseData.orientation[0],
		                      (float)poseData.orientation[1],
		                      (float)poseData.orientation[2],
		                      (float)poseData.orientation[3]);
		m_imuTDepthCamera = Matrix4x4.TRS(position, quat, new Vector3(1.0f, 1.0f, 1.0f));
		
		// Query the extrinsics between IMU and camera color frame.
		pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
		pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_CAMERA_COLOR;
		position = new Vector3((float)poseData.translation[0],
		                       (float)poseData.translation[1],
		                       (float)poseData.translation[2]);
		quat = new Quaternion((float)poseData.orientation[0],
		                      (float)poseData.orientation[1],
		                      (float)poseData.orientation[2],
		                      (float)poseData.orientation[3]);
		m_imuTColorCamera = Matrix4x4.TRS(position, quat, new Vector3(1.0f, 1.0f, 1.0f));
		
		// Query the extrinsics betwween Depth Frame and Color Frame
		m_ccTDepth = Matrix4x4.Inverse (m_imuTColorCamera) * m_imuTDepthCamera;
	}
	
	// Get camera Intrinsics
	private void _SetCameraIntrinsics()
	{
		VideoOverlayProvider.GetIntrinsics(TangoEnums.TangoCameraId.TANGO_CAMERA_COLOR, intrinsics);
	}
	
}