set pagination off
set print pretty off
set print elements 0

define gimbal_snapshot
    printf "\n=== Gimbal debug snapshot ===\n"
    printf "tick: uwTick=%u\n", uwTick

    printf "\n[vision rx]\n"
    printf "packets=%u bytes=%u valid=%u crc_err=%u latest_valid=%u latest_pending=%u seq=%u seq_echo=%u target_valid=%u\n", vision_debug.usb_rx_packet_count, vision_debug.usb_rx_byte_count, vision_debug.valid_frame_count, vision_debug.crc_error_count, vision_debug.latest_cmd_valid, vision_debug.latest_cmd_pending, vision_debug.last_seq, vision_debug.seq_echo, vision_debug.last_target_valid
    printf "delta_raw_1e4rad: yaw=%d pitch=%d\n", vision_debug.last_delta_yaw_1e4rad, vision_debug.last_delta_pitch_1e4rad
    printf "delta_rad: yaw=%g pitch=%g\n", vision_debug.last_delta_yaw_rad, vision_debug.last_delta_pitch_rad
    printf "delta_deg: yaw=%g pitch=%g\n", vision_debug.last_delta_yaw_deg, vision_debug.last_delta_pitch_deg
    printf "actual_rad: yaw=%g pitch=%g\n", vision_debug.actual_yaw_rad, vision_debug.actual_pitch_rad
    printf "actual_deg: yaw=%g pitch=%g\n", vision_debug.actual_yaw_deg, vision_debug.actual_pitch_deg
    printf "crc: calc=0x%04x recv=0x%04x\n", vision_debug.last_calc_crc, vision_debug.last_recv_crc
    printf "last_usb_packet: "
    x/10xb &vision_debug.last_usb_packet_bytes[0]
    printf "last_valid_frame: "
    x/10xb &vision_debug.last_valid_frame[0]

    printf "\n[robot cmd]\n"
    printf "robot_state=%d\n", 'Application/cmd/robot_cmd.c'::robot_state
    printf "ready_flags: imu=%u yaw_motor=%u pitch_motor=%u\n", 'Application/cmd/robot_cmd.c'::gimbal_fetch_data.imu_online, 'Application/cmd/robot_cmd.c'::gimbal_fetch_data.yaw_motor_online, 'Application/cmd/robot_cmd.c'::gimbal_fetch_data.pitch_motor_online
    printf "cmd_send: mode=%d yaw=%g pitch=%g\n", ('Application/cmd/robot_cmd.c'::gimbal_cmd_send).gimbal_mode, ('Application/cmd/robot_cmd.c'::gimbal_cmd_send).yaw, ('Application/cmd/robot_cmd.c'::gimbal_cmd_send).pitch
    printf "vision_target: valid=%u yaw=%g pitch=%g\n", 'Application/cmd/robot_cmd.c'::vision_target_valid, 'Application/cmd/robot_cmd.c'::vision_target_yaw_rad, 'Application/cmd/robot_cmd.c'::vision_target_pitch_rad
    printf "sentry: state=%u cmd_ready=%u cmd_target_valid=%u stall_axis=%u stall_detected=%u scan_yaw=%g scan_pitch=%g recovery_yaw=%g recovery_pitch=%g\n", robot_cmd_debug.sentry_state, robot_cmd_debug.vision_cmd_ready, robot_cmd_debug.vision_cmd_target_valid, robot_cmd_debug.stall_axis, robot_cmd_debug.stall_detected, robot_cmd_debug.scan_yaw_target_rad, robot_cmd_debug.scan_pitch_target_rad, robot_cmd_debug.recovery_yaw_target_rad, robot_cmd_debug.recovery_pitch_target_rad

    printf "\n[gimbal task]\n"
    printf "cmd_recv: mode=%d yaw=%g pitch=%g\n", ('Application/gimbal/gimbal.c'::gimbal_cmd_recv).gimbal_mode, ('Application/gimbal/gimbal.c'::gimbal_cmd_recv).yaw, ('Application/gimbal/gimbal.c'::gimbal_cmd_recv).pitch
    printf "zero: yaw_locked=%u yaw_offset=%g pitch_locked=%u pitch_offset=%g\n", 'Application/gimbal/gimbal.c'::yaw_zero_locked, 'Application/gimbal/gimbal.c'::yaw_zero_offset_rad, 'Application/gimbal/gimbal.c'::pitch_zero_locked, 'Application/gimbal/gimbal.c'::pitch_zero_offset_rad
    printf "feedback_axis: yaw_angle=%g yaw_speed=%g pitch_angle=%g pitch_speed=%g\n", 'Application/gimbal/gimbal.c'::yaw_angle_feedback_rad, 'Application/gimbal/gimbal.c'::yaw_speed_feedback, 'Application/gimbal/gimbal.c'::pitch_angle_feedback_rad, 'Application/gimbal/gimbal.c'::pitch_speed_feedback

    printf "\n[motor registry]\n"
    printf "gm6020_count=%u\n", gm6020_count
    set $yaw = 'Application/gimbal/gimbal.c'::yaw_motor
    if $yaw == 0
        printf "yaw_motor=NULL\n"
    else
        printf "yaw_motor: id=%u enabled=%u ref=%g fb=%g err=%g speed_ref=%g speed_fb=%g current_ref=%g current_fb=%g volt_ref=%g ff=%g output=%d last_rx=%u\n", $yaw->motor_id, $yaw->enabled, $yaw->angle_ref_rad, $yaw->angle_feedback_rad, $yaw->angle_ref_rad - $yaw->angle_feedback_rad, $yaw->speed_ref_rad_s, $yaw->speed_feedback_rad_s, $yaw->current_ref_raw, $yaw->current_feedback_raw, $yaw->voltage_ref_raw, $yaw->output_ff_raw, $yaw->output_cmd, $yaw->last_rx_tick
        printf "yaw_pid: angle_out=%g speed_out=%g current_out=%g\n", $yaw->angle_pid.Output, $yaw->speed_pid.Output, $yaw->current_pid.Output
    end
    set $pitch = 'Application/gimbal/gimbal.c'::pitch_motor
    if $pitch == 0
        printf "pitch_motor=NULL\n"
    else
        printf "pitch_motor: id=%u enabled=%u ref=%g fb=%g err=%g speed_ref=%g speed_fb=%g current_ref=%g current_fb=%g volt_ref=%g ff=%g output=%d last_rx=%u\n", $pitch->motor_id, $pitch->enabled, $pitch->angle_ref_rad, $pitch->angle_feedback_rad, $pitch->angle_ref_rad - $pitch->angle_feedback_rad, $pitch->speed_ref_rad_s, $pitch->speed_feedback_rad_s, $pitch->current_ref_raw, $pitch->current_feedback_raw, $pitch->voltage_ref_raw, $pitch->output_ff_raw, $pitch->output_cmd, $pitch->last_rx_tick
        printf "pitch_pid: angle_out=%g speed_out=%g current_out=%g\n", $pitch->angle_pid.Output, $pitch->speed_pid.Output, $pitch->current_pid.Output
    end

    printf "\n[can tx]\n"
    printf "tx_attempt=%u tx_success=%u tx_fail=%u tx_abort=%u last_tick=%u std_id=0x%03x group=%u hal_err=0x%x can_err=0x%x free=%u mailbox=%u\n", gm6020_debug.tx_attempt_count, gm6020_debug.tx_success_count, gm6020_debug.tx_fail_count, gm6020_debug.tx_abort_count, gm6020_debug.last_tx_tick_ms, gm6020_debug.last_tx_std_id, gm6020_debug.last_tx_group, gm6020_debug.last_hal_error, gm6020_debug.last_can_error_code, gm6020_debug.last_tx_free_level, gm6020_debug.last_tx_mailbox
    printf "last_tx_data: "
    x/8xb &gm6020_debug.last_tx_data[0]
    printf "last_output_cmd_by_index: "
    x/8hd &gm6020_debug.last_output_cmd[0]

    printf "\n[sysid]\n"
    printf "mode=%u phase=%u seq=%u elapsed_ms=%u yaw_offset=%g pitch_offset=%g target_offset=%g\n", gimbal_sysid_debug.mode, gimbal_sysid_debug.phase, gimbal_sysid_debug.seq_index, gimbal_sysid_debug.elapsed_ms, gimbal_sysid_debug.yaw_offset_rad, gimbal_sysid_debug.pitch_offset_rad, gimbal_sysid_debug.target_offset_rad
    printf "=== end snapshot ===\n\n"
end

document gimbal_snapshot
Print one compact live snapshot of the vision RX -> robot command -> gimbal task -> PID/motor -> CAN TX chain.
Run after connecting GDB to the target.
end
