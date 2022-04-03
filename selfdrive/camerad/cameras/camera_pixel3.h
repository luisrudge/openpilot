#pragma once

#include <cstdint>

#include <media/cam_req_mgr.h>

#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/common/util.h"

//#define USE_V4L2_EVENT_FOR_CAM_SYNC

#ifdef USE_V4L2_EVENT_FOR_CAM_SYNC
#include <map>
#include <media_pixel3/cam_sync.h>
#endif

#define FRAME_BUF_COUNT 4

#ifdef USE_V4L2_EVENT_FOR_CAM_SYNC
struct CameraRequestInfo {
public:
  CameraRequestInfo() {
  }
  void set_sync_status(int status) { sync_status = status;}
  void set_sof_result(struct cam_req_mgr_message *m) {
    sof_msg = *m;
    sof_ready = true;
  }
  bool finished() {
    return sync_status == CAM_SYNC_STATE_SIGNALED_SUCCESS && sof_ready;
  }
  bool sync_error() { return sync_status == CAM_SYNC_STATE_SIGNALED_ERROR;}
  int32_t sync_obj;
  int32_t sync_status = CAM_SYNC_STATE_INVALID;
  struct cam_req_mgr_message sof_msg;
  bool sof_ready = false;
};
#endif

class CameraState {
public:
  MultiCameraState *multi_cam_state;
  CameraInfo ci;

  std::mutex exp_lock;

  int exposure_time;
  bool dc_gain_enabled;
  float analog_gain_frac;

  float cur_ev[3];
  float min_ev, max_ev;

  float measured_grey_fraction;
  float target_grey_fraction;
  int gain_idx;

  unique_fd sensor_fd;
  unique_fd csiphy_fd;

  int camera_num;

  void config_isp(int io_mem_handle, int fence, int request_id, int buf0_mem_handle, int buf0_offset);
  void enqueue_req_multi(int start, int n, bool dp);
  void enqueue_buffer(int i, bool dp);
  void handle_camera_event(void *evdat);
  void set_camera_exposure(float grey_frac);

  void sensors_start();
  void sensors_poke(int request_id);
  void sensors_i2c(struct i2c_random_wr_payload* dat, int len, int op_code);
  void sensors_init();

  void camera_open();
  void camera_init(MultiCameraState *multi_cam_state, VisionIpcServer * v, int camera_id, int camera_num, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type);
  void camera_close();

  int32_t session_handle;
  int32_t sensor_dev_handle;
  int32_t isp_dev_handle;
  int32_t csiphy_dev_handle;

  int32_t link_handle;

  int buf0_handle;
  int buf_handle[FRAME_BUF_COUNT];
  int sync_objs[FRAME_BUF_COUNT];
  int request_ids[FRAME_BUF_COUNT];
  int request_id_last;
  int frame_id_last;
  int idx_offset;
  bool skipped;

  CameraBuf buf;
#ifdef USE_V4L2_EVENT_FOR_CAM_SYNC
  bool handle_camera_sync_event(struct cam_sync_ev_header *event_data);
  void handle_req_finished(int req_id);
  void reset_all_maps();

  std::map<int, CameraRequestInfo> req_info_map;
  std::map<int, int> sync_obj_to_req_id;
#endif

};

typedef struct MultiCameraState {
  unique_fd video0_fd;
  unique_fd video1_fd;
  unique_fd isp_fd;
  int device_iommu;
  int cdm_iommu;


  CameraState road_cam;
  CameraState driver_cam;
  SubMaster *sm;
  PubMaster *pm;
} MultiCameraState;
