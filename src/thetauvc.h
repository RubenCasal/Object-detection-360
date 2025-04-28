#ifndef __THETAUVC_H__
#define __THETAUVC_H__

#ifdef __cplusplus
extern "C" {
#endif

enum thetauvc_mode_code {
    THETAUVC_MODE_UHD_2997 = 0,
    THETAUVC_MODE_FHD_2997,
    THETAUVC_MODE_NUM
};

extern uvc_error_t thetauvc_find_devices(uvc_context_t *, uvc_device_t ***);
extern uvc_error_t thetauvc_print_devices(uvc_context_t *, FILE *);
extern uvc_error_t thetauvc_find_device(uvc_context_t *, uvc_device_t **, unsigned int);
extern uvc_error_t thetauvc_get_stream_ctrl_format_size(uvc_device_handle_t *, unsigned int, uvc_stream_ctrl_t *);
extern uvc_error_t thetauvc_run_streaming(uvc_device_t *, uvc_device_handle_t **, unsigned int, uvc_frame_callback_t *, void *);

#ifdef __cplusplus
}
#endif

#endif  // __THETAUVC_H__
