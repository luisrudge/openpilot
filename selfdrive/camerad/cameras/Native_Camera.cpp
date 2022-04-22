#include "selfdrive/camerad/cameras/Native_Camera.h"
#include <cassert>

// TODO - reasonable actions with callbacks
// Camera Callbacks
static void CameraDeviceOnDisconnected(void* context, ACameraDevice* device) {
  LOGD("Camera(id: %s) is diconnected.\n", ACameraDevice_getId(device));
}
static void CameraDeviceOnError(void* context, ACameraDevice* device,
                                int error) {
  LOGE("Error(code: %d) on Camera(id: %s).\n", error,
       ACameraDevice_getId(device));
}
// Capture Callbacks
static void CaptureSessionOnReady(void* context,
                                  ACameraCaptureSession* session) {
  LOGD("Session is ready.\n");
}
static void CaptureSessionOnActive(void* context,
                                   ACameraCaptureSession* session) {
  LOGD("Session is activated.\n");
}

Native_Camera::Native_Camera(camera_type type) {

  ACameraMetadata* cameraMetadata = nullptr;
  camera_status_t cameraStatus = ACAMERA_OK;

  m_camera_manager = ACameraManager_create();

  cameraStatus =
      ACameraManager_getCameraIdList(m_camera_manager, &m_camera_id_list);
  assert(cameraStatus == ACAMERA_OK);
  assert(m_camera_id_list->numCameras > 0);

  // ASSUMPTION: Back camera is index[0] and front is index[1]
  // TODO - why I need orientation as is below
  if (type == BACK_CAMERA) {
    m_selected_camera_id = m_camera_id_list->cameraIds[0];
    m_camera_orientation = 90;
  } else {
    assert(m_camera_id_list->numCameras > 1);
    m_selected_camera_id= m_camera_id_list->cameraIds[1];
    m_camera_orientation = 270;
  }

  cameraStatus = ACameraManager_getCameraCharacteristics(
      m_camera_manager, m_selected_camera_id, &cameraMetadata);
  assert(cameraStatus == ACAMERA_OK);

  m_device_state_callbacks.onDisconnected = CameraDeviceOnDisconnected;
  m_device_state_callbacks.onError = CameraDeviceOnError;

  cameraStatus =
      ACameraManager_openCamera(m_camera_manager, m_selected_camera_id,
                                &m_device_state_callbacks, &m_camera_device);
  assert(cameraStatus == ACAMERA_OK);

//  ACameraMetadata_const_entry entry;
//  ACameraMetadata_getConstEntry(
//      cameraMetadata, ACAMERA_INFO_SUPPORTED_HARDWARE_LEVEL, &entry);

  m_camera_ready = true;
}

Native_Camera::~Native_Camera() {
  if (m_capture_request != nullptr) {
    ACaptureRequest_free(m_capture_request);
    m_capture_request = nullptr;
  }

  if (m_camera_output_target != nullptr) {
    ACameraOutputTarget_free(m_camera_output_target);
    m_camera_output_target = nullptr;
  }

  if (m_camera_device != nullptr) {
    ACameraDevice_close(m_camera_device);
    m_camera_device = nullptr;
  }

  ACaptureSessionOutputContainer_remove(m_capture_session_output_container,
                                        m_session_output);
  if (m_session_output != nullptr) {
    ACaptureSessionOutput_free(m_session_output);
    m_session_output = nullptr;
  }

  if (m_capture_session_output_container != nullptr) {
    ACaptureSessionOutputContainer_free(m_capture_session_output_container);
    m_capture_session_output_container = nullptr;
  }

  ACameraManager_delete(m_camera_manager);
}

bool Native_Camera::MatchCaptureSizeRequest(ImageFormat* resView, int32_t width, int32_t height) {
  Display_Dimension disp(width, height);
  if (m_camera_orientation == 90 || m_camera_orientation == 270) {
    disp.Flip();
  }

  ACameraMetadata* metadata;
  ACameraManager_getCameraCharacteristics(m_camera_manager,
                                          m_selected_camera_id, &metadata);
  ACameraMetadata_const_entry entry;
  ACameraMetadata_getConstEntry(
      metadata, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);
  // format of the data: format, width, height, input?, type int32
  bool foundIt = false;
  Display_Dimension foundRes(1000, 1000); // max resolution for current gen phones

  for (int i = 0; i < entry.count; ++i) {
    int32_t input = entry.data.i32[i * 4 + 3];
    int32_t format = entry.data.i32[i * 4 + 0];
    if (input) continue;
    Display_Dimension res(entry.data.i32[i * 4 + 1],
                           entry.data.i32[i * 4 + 2]);
    if(format != 0)
      LOGD("fmt: 0x%X, w: %d, h: %d", format, res.width(), res.height());
    if (format == AIMAGE_FORMAT_RAW10) {
      if (!disp.IsSameRatio(res)) continue;
      if (foundRes > res) {
        foundIt = true;
        foundRes = res;
      }
    }
  }

  if (foundIt) {
    resView->width = foundRes.org_width();
    resView->height = foundRes.org_height();
  } else {
    if (disp.IsPortrait()) {
      resView->width = 480;
      resView->height = 640;
    } else {
      resView->width = 640;
      resView->height = 480;
    }
  }
  resView->format = AIMAGE_FORMAT_YUV_420_888;
  LOGD("--- W -- H -- %d -- %d",resView->width,resView->height);
  return foundIt;
}

bool Native_Camera::CreateCaptureSession(ANativeWindow* window) {

  camera_status_t cameraStatus = ACAMERA_OK;

  ACaptureSessionOutputContainer_create(&m_capture_session_output_container);
  ANativeWindow_acquire(window);
  ACaptureSessionOutput_create(window, &m_session_output);
  ACaptureSessionOutputContainer_add(m_capture_session_output_container,
                                     m_session_output);
  ACameraOutputTarget_create(window, &m_camera_output_target);

  // TEMPLATE_RECORD because rather have post-processing quality for more
  // accureate CV algo
  // Frame rate should be good since all image buffers are being done from
  // native side
  cameraStatus = ACameraDevice_createCaptureRequest(m_camera_device,
                                                    TEMPLATE_RECORD, &m_capture_request);
  LOGD("ACameraDevice_createCaptureRequest status %d", cameraStatus);
  if(cameraStatus != ACAMERA_OK)
    return false;

  ACaptureRequest_addTarget(m_capture_request, m_camera_output_target);

  m_capture_session_state_callbacks.onReady = CaptureSessionOnReady;
  m_capture_session_state_callbacks.onActive = CaptureSessionOnActive;
  ACameraDevice_createCaptureSession(
      m_camera_device, m_capture_session_output_container,
      &m_capture_session_state_callbacks, &m_capture_session);

  ACameraCaptureSession_setRepeatingRequest(m_capture_session, nullptr, 1,
                                            &m_capture_request, nullptr);

  return true;
}
