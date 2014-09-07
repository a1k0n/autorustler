#include <bcm_host.h>
#include <stdio.h>
#include <stdlib.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

static const char OUTPUT_FILENAME[] = "output.yuv";
MMAL_POOL_T *camera_pool = NULL;
FILE *output_file = NULL;
int frame_number = 0;

static void camera_control_callback(
    MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  fprintf(stderr, "Camera control callback  cmd=0x%08x", buffer->cmd);

  mmal_buffer_header_release(buffer);
}

static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port,
                                   MMAL_PORT_T *input_port,
                                   MMAL_CONNECTION_T **connection) {
  MMAL_STATUS_T status;

  status = mmal_connection_create(
      connection, output_port, input_port,
      MMAL_CONNECTION_FLAG_TUNNELLING |
      MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

  if (status == MMAL_SUCCESS) {
    status = mmal_connection_enable(*connection);
    if (status != MMAL_SUCCESS)
      mmal_connection_destroy(*connection);
  }

  return status;
}

static void camera_buffer_callback(MMAL_PORT_T *port,
                                   MMAL_BUFFER_HEADER_T *buffer) {
  int complete = 0;

  int bytes_written = buffer->length;

  fprintf(stderr, "camera buffer callback: %d bytes\n", buffer->length);

  if (buffer->length) {
    mmal_buffer_header_mem_lock(buffer);
    bytes_written = fwrite(buffer->data, 1, buffer->length, output_file);
    mmal_buffer_header_mem_unlock(buffer);
  }

  // We need to check we wrote what we wanted - it's possible we have run out of storage.
  if (bytes_written != buffer->length) {
    fprintf(stderr, "Unable to write buffer to file - aborting\n");
    complete = 1;
  }

  // Check end of frame or error
  if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END |
                       MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)) {
    complete = 1;
    fprintf(stderr, "frame %d complete\n", frame_number++);
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status;
    MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(camera_pool->queue);

    // and back to the port from there.
    if (new_buffer) {
      status = mmal_port_send_buffer(port, new_buffer);
    }

    if (!new_buffer || status != MMAL_SUCCESS)
      fprintf(stderr, "Unable to return the buffer to the "
              "camera still port\n");
  }

  // if (complete) {
  //   vcos_semaphore_post(&(pData->complete_semaphore));
  // }
  // can signal main program here, but... not sure what to do yet
}

int main(int argc, const char **argv) {
  bcm_host_init();

  MMAL_COMPONENT_T *camera = NULL;
  MMAL_STATUS_T status;

  if ((status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera))
      != MMAL_SUCCESS) {
    fprintf(stderr, "cannot create camera :(\n");
    return 1;
  }

  // Enable the camera, and tell it its control callback function
  status = mmal_port_enable(camera->control, camera_control_callback);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot enable camera\n");
    return 1;
  }

  printf("woo, created camera with %d outputs, now what?\n",
         camera->output_num);

  MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
    { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
    .max_stills_w = 400,
    .max_stills_h = 300,
    .stills_yuv422 = 0,
    .one_shot_stills = 1,
    .max_preview_video_w = 400,
    .max_preview_video_h = 300,
    .num_preview_video_frames = 3,
    .stills_capture_circular_buffer_height = 0,
    .fast_preview_resume = 0,
    .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
  };

  mmal_port_parameter_set(camera->control, &cam_config.hdr);

  // TODO: look at all the junk set here:
  // raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

  MMAL_PORT_T *preview_port = camera->output[0];
  MMAL_PORT_T *video_port = camera->output[1];
  MMAL_PORT_T *still_port = camera->output[2];

  MMAL_ES_FORMAT_T *format = preview_port->format;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;
  format->es->video.width = 416;
  format->es->video.height = 304;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = 400;
  format->es->video.crop.height = 300;
  format->es->video.frame_rate.num = 0;
  format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(preview_port);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set preview port format\n");
    return 1;
  }

  mmal_format_full_copy(video_port->format, format);
  // Ensure there are enough buffers to avoid dropping frames
  status = mmal_port_format_commit(video_port);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set video port format\n");
    return 1;
  }

  if (video_port->buffer_num < 3)
    video_port->buffer_num = 3;

  format = still_port->format;
  // format->encoding = MMAL_ENCODING_I420;
  format->encoding = MMAL_ENCODING_BGR24;
  format->encoding_variant = MMAL_ENCODING_BGR24;
  format->es->video.width = 416;
  format->es->video.height = 304;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = 400;
  format->es->video.crop.height = 300;
  format->es->video.frame_rate.num = 0;
  format->es->video.frame_rate.den = 1;

  if (still_port->buffer_size < still_port->buffer_size_min)
    still_port->buffer_size = still_port->buffer_size_min;
  still_port->buffer_num = still_port->buffer_num_recommended;
  status = mmal_port_format_commit(still_port);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set still port format\n");
    return 1;
  }

  status = mmal_component_enable(camera);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot enable camera\n");
    return 1;
  }
  // the red LED should be on now.

  fprintf(stderr, "still port %s: %d buffers x %d bytes\n",
          still_port->name, still_port->buffer_num, still_port->buffer_size);

  // Create pool of buffer headers for the output port to consume
  camera_pool = mmal_port_pool_create(
      still_port, still_port->buffer_num, still_port->buffer_size);
  if (!camera_pool) {
    fprintf(stderr, "cannot create buffer header pool for still port\n");
    return 1;
  }

  // create a null sink for preview, which we don't need
  MMAL_COMPONENT_T *nullsink = NULL;
  status = mmal_component_create("vc.null_sink", &nullsink);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot create null sink\n");
    return 1;
  }
  status = mmal_component_enable(nullsink);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot enable null sink\n");
    return 1;
  }

  output_file = fopen(OUTPUT_FILENAME, "wb");
  if (!output_file) {
    perror(OUTPUT_FILENAME);
    return 1;
  }

  fprintf(stderr, "Starting component connection stage\n");
  MMAL_CONNECTION_T *preview_connection;
  status = connect_ports(preview_port, nullsink->input[0],
                         &preview_connection);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot connect null sink input to preview port\n");
    return 1;
  }

  status = mmal_port_enable(still_port, camera_buffer_callback);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "Failed to setup camera output\n");
    return 1;
  }

  fprintf(stderr, "Starting video preview\n");

  // set shutter speed to auto (it probably was already?)
  if (mmal_port_parameter_set_uint32(
              camera->control, MMAL_PARAMETER_SHUTTER_SPEED,
              0) != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set shutter speed\n");
    return 1;
  }

  // Send all the buffers to the camera output port
  int qlen = mmal_queue_length(camera_pool->queue);
  for (int q = 0; q < qlen; q++) {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(camera_pool->queue);
    if (!buffer) {
      fprintf(stderr,
              "Unable to get a required buffer %d from pool queue\n",
              q);
    }

    if (mmal_port_send_buffer(still_port, buffer)!= MMAL_SUCCESS) {
      fprintf(stderr,
              "Unable to send a buffer to camera output port (%d)\n",
              q);
    }
  }

  // TODO: burst capture?

  sleep(1);  // wait one second for auto-exposure

  // enable capturing
  if (mmal_port_parameter_set_boolean(still_port, MMAL_PARAMETER_CAPTURE, 1)
      != MMAL_SUCCESS) {
    fprintf(stderr, "failed to start capture\n");
    return 1;
  }
  fprintf(stderr, "started capture 1\n");

  sleep(1);

  // enable capturing
  if (mmal_port_parameter_set_boolean(still_port, MMAL_PARAMETER_CAPTURE, 1)
      != MMAL_SUCCESS) {
    fprintf(stderr, "failed to start capture\n");
    return 1;
  }
  fprintf(stderr, "started capture 2\n");

  sleep(1);

  return 0;
}
