# Abbreviations

The table Barr-C 1.5.b asks for: abbreviations in subsystem 1's code and
messages that are not common engineering terms. The files named are in
`app_program/subsystem1/` or `testing/subsystem1/`. What Appendix A of the
standard already accepts, such as `msg` and `buf`, is not repeated here.

| Abbreviation | Meaning | Where it is used |
|---|---|---|
| ack | Acknowledgement | `ack_publish_failed`, `s1_telem_test.c` |
| alloc | Allocate | `alloc_copy`, `codec_test_main.c` |
| bc | Barcode | `bc_letter`, `msg_codec.h` |
| bool | Boolean, C `stdbool.h` type | `bool`, `codec_vectors.h` |
| c39 | Code 39, the barcode symbology | `S3_C39_IDX_NONE`, `telemetry_task.c` |
| cal | Calibration | `b_cal_done`, `s1_telem.h` |
| cdeg | Centidegrees, hundredths of a degree | `heading_cdeg`, `s1_telem.h` |
| ch | Character, a Code 39 letter | `last_ch`, `s1_telem.h` |
| cmd | Command | `last_cmd`, `s1_telem.h` |
| codec | Coder-decoder | `msg_codec_live_t`, `codec_vectors.h` |
| ctrl | Controller | `ctrl_state`, `s1_telem.h` |
| deg | Degrees | `servo_deg`, `s1_telem.h` |
| desc | Descriptor | `msg_desc_t`, `codec_test_main.c` |
| dir | Direction | `b_bc_dir`, `s1_telem.c` |
| dist | Distance | `dist_mm`, `s1_telem.h` |
| dps | Degrees per second | `turn_rate_dps`, `s1_telem.h` |
| dv | Wheel speed difference, mm/s | `steer_dv_mms`, `msg_codec.h` |
| e | Error, kernel code prefix | `E_OK`, `telemetry_task.c` |
| estop | Emergency stop | `b_estop`, `codec_vectors.c` |
| evt | Event | `evt_drop`, `msg_codec.h` |
| frbufsz | Free buffer size, in `T_RMBF` | `frbufsz`, `mqtt_queue.c` |
| gb | Global Boolean, `g_` plus `b_` | `gb_ready`, `mqtt_queue.c` |
| gh | Global handle, `g_` plus `h_` | `gh_publish_mbf`, `mqtt_queue.c` |
| hb | Heartbeat | `S1_TELEM_HB_QUEUED_MAX`, `s1_telem.h` |
| hdeg | Half-degrees | `heading_hdeg`, `msg_codec.h` |
| hdr | Header | `HDR_HEARTBEAT`, `s1_telem_test.c` |
| idx | Index | `angle_idx`, `s1_telem.c` |
| imu | Inertial measurement unit | `b_imu_ok`, `s1_telem.h` |
| int16 | Signed 16-bit, C `stdint.h` type | `int16_t`, `msg_codec.h` |
| int32 | Signed 32-bit, C `stdint.h` type | `int32_t`, `codec_vectors.h` |
| ir | Infra-red | `outer_ir`, `msg_codec.h` |
| junc | Junction | `junc_turn`, `msg_codec.h` |
| l | Left | `count_l`, `s1_telem.h` |
| len | Length | `buf_len`, `msg_codec.c` |
| liv | Live message (type 6) | `MSG_LIV_BYTES`, `msg_layout.h` |
| losts | Line-lost count, S3's field | `losts`, `msg_codec.h` |
| mbf | Message buffer | `gh_publish_mbf`, `mqtt_queue.c` |
| memset | C library: set memory | `memset`, `s1_telem_test.c` |
| mm | Millimetres | `dist_mm`, `s1_telem.h` |
| mms | Millimetres per second | `cmd_speed_mms`, `msg_codec.h` |
| mqtt | MQTT messaging protocol | `mqtt_link_publish`, `telemetry_task.c` |
| msgs | Messages | `msgs`, `codec_test_main.c` |
| n | Count; see the note on S3's `n_outer` | `n_cmd`, `codec_test_main.c` |
| obst | Obstacle | `obst`, `telemetry_task.c` |
| odom | Odometry | `subsystem2_odom_t`, `telemetry_task.c` |
| ok | Okay, no error | `b_ok`, `msg_codec.c` |
| pb | Pointer to Boolean, `p_` plus `b_` | `pb_fault_set`, `s1_telem.c` |
| pos | Bit position | `MSG_LIV_SEQ_POS`, `msg_layout.h` |
| pub | Publish | `s1_telem_pub_t`, `telemetry_task.c` |
| qos | MQTT quality of service | `MQTT_LINK_QOS_0`, `telemetry_task.c` |
| r | Right | `count_r`, `s1_telem.h` |
| ref | Reference (read status), kernel call | `tk_ref_mbf`, `mqtt_queue.c` |
| rev | Reversed, read backwards | `b_last_rev`, `s1_telem.h` |
| rmbf | Kernel message buffer status | `rmbf`, `mqtt_queue.c` |
| s | Seconds | `uptime_s`, `telemetry_task.c` |
| s1 | Subsystem 1 | `s1_telem_src_t`, `s1_telem.h` |
| s16 | Signed 16-bit | `get_s16`, `msg_codec.c` |
| s2 | Subsystem 2 | `S2_STATE_IDLE`, `telemetry_task.c` |
| s3 | Subsystem 3 | `S3_CMD_LEFT`, `telemetry_task.c` |
| s4 | Subsystem 4 | `S4_CLASS_IMPACT`, `telemetry_task.c` |
| s5 | Subsystem 5 | `S5_FAULT_IMU`, `telemetry_task.c` |
| sat | Saturating | `s1_telem_sat_add`, `s1_telem.c` |
| seq | Sequence counter | `g_hb_seq`, `telemetry_task.c` |
| src | Source | `s1_telem_src_t`, `s1_telem.h` |
| stats | Statistics, the counters | `g_stats`, `telemetry_task.c` |
| t | Type | `s1_telem_src_t`, `s1_telem.h` |
| tel | Telemetry | `tel_drop`, `msg_codec.h` |
| telem | Telemetry | `s1_telem_due`, `s1_telem.c` |
| tk | T-Kernel, kernel call prefix | `tk_ref_mbf`, `mqtt_queue.c` |
| u8 | Unsigned 8-bit | `get_u8`, `msg_codec.c` |
| uint16 | Unsigned 16-bit, C `stdint.h` type | `uint16_t`, `s1_telem.h` |
| uint32 | Unsigned 32-bit, C `stdint.h` type | `uint32_t`, `codec_vectors.c` |
| uint8 | Unsigned 8-bit, C `stdint.h` type | `uint8_t`, `codec_vectors.h` |
| uncal | Uncalibrated | `S1_TELEM_LINE_UNCAL`, `s1_telem.h` |
| uturn | U-turn | `MSG_BC_CMD_UTURN`, `msg_layout.h` |
| vec | Vector | `vec_count`, `codec_test_main.c` |
| wifi | Wi-Fi | `MSG_FAULT_WIFI`, `telemetry_task.c` |

`dv` is S3's term: `app_program/subsystem3/s3_api.h` (IF-02) defines it as the
wheel speed difference in mm/s, with + meaning turn right.

The `n` in S3's `n_outer` and `n_centre` is per S3, to confirm: S3 calls them
IR readings and does not define the letter.

## Proposed deviations

Barr-C 1.5.a says abbreviations should generally be avoided. These names are
kept because they match S2 and S4 field names and the wire layout.

| Name | Where it matches | Status |
|---|---|---|
| mms | S2's `speed_l_mms` and `target_l_mms` | proposed, awaiting approval |
| dps | S4's `turn_rate_dps` | proposed, awaiting approval |
| uptime_s | The heartbeat's `uptime_s` field | proposed, awaiting approval |
