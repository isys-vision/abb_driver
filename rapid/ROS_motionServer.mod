MODULE ROS_motionServer

! Software License Agreement (BSD License)
!
! Copyright (c) 2012, Edward Venator, Case Western Reserve University
! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
! All rights reserved.
!
! Redistribution and use in source and binary forms, with or without modification,
! are permitted provided that the following conditions are met:
!
!   Redistributions of source code must retain the above copyright notice, this
!       list of conditions and the following disclaimer.
!   Redistributions in binary form must reproduce the above copyright notice, this
!       list of conditions and the following disclaimer in the documentation
!       and/or other materials provided with the distribution.
!   Neither the name of the Case Western Reserve University nor the names of its contributors
!       may be used to endorse or promote products derived from this software without
!       specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

LOCAL CONST num server_port := 11000;

LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;
LOCAL VAR num trajectory_size;
VAR clock clk_traj;
VAR num traj_time;
VAR MIKADO_connection_info connection_info;
VAR jointtarget point;


PROC main()
    VAR ROS_msg_joint_traj_pt message;
    ROS_trajectory_lock := FALSE;

    TPWrite "[MotionServer] Waiting for connection.";
    ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket, "MotionServer";

    ! wait for connection info message and configure driver
    MIKADO_receive_connection_info client_socket, connection_info;
    IF(isDebug) THEN
        TPWrite "[MotionServer] Received connection info. Num traj axes: " + ValToStr(connection_info.num_ax_traj);
    ENDIF
    connection_info.traj_msg_length := 8 + 4 * (connection_info.num_ax_traj + connection_info.num_eax_traj);
    IF(connection_info.is_traj_duration) THEN
        connection_info.traj_msg_length := connection_info.traj_msg_length + 4;
    ENDIF
    IF(connection_info.is_traj_velocity) THEN
        connection_info.traj_msg_length := connection_info.traj_msg_length + 4;
    ENDIF

    ! main loop -> receive trajectory points and process them
    WHILE ( true ) DO
        ROS_receive_msg_dyn_jts_traj_pt client_socket, message;
        trajectory_pt_callback message;
    ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED, ERR_SOCK_UNSPEC)
    IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) OR (ERRNO=ERR_SOCK_UNSPEC) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "[MotionServer] ROS MotionServer disconnect", "Connection lost.  Resetting socket.";
        ! restart program
        traj_size_pers:=0;
        ExitCycle;
    ELSE
        ErrWrite \W, "[MotionServer] ROS MotionServer unhandled error", "Try again.";
        ! empty trajectory
        traj_size_pers:=0;
        trajectory_size := 0;
        activate_trajectory;
        ! issue stop command just to be safe
        StopMove; ClearPath; StartMove;
        TRYNEXT;
    ENDIF
UNDO
    IF (SocketGetStatus(client_socket) <> SOCKET_CLOSED) SocketClose client_socket;
    IF (SocketGetStatus(server_socket) <> SOCKET_CLOSED) SocketClose server_socket;
ENDPROC

LOCAL PROC trajectory_pt_callback(ROS_msg_joint_traj_pt message)
    VAR jointtarget point;
    VAR ROS_msg reply_msg;
    VAR num velocity;
    point := [message.joints, message.ext_axes];
    ! velocity is not taking into account, all points are driven with the same fixed velocity
    velocity := message.velocity;

    ! use sequence_id to signal start/end of trajectory download
    TEST message.sequence_id
        CASE ROS_TRAJECTORY_START_DOWNLOAD:
            is_not_receiving_traj := FALSE;
            IF(isDebug) THEN
                ClkReset clk_traj;
                ClkStart clk_traj;
                TPWrite "[MotionServer] Traj START received";
            ENDIF
            trajectory_size := 0; ! reset trajectory size
            add_traj_pt point, velocity;
        CASE ROS_TRAJECTORY_END:
            add_traj_pt point, velocity;
            activate_trajectory;
            IF(isDebug) THEN
                TPWrite "[MotionServer] Traj END received";
                ClkStop clk_traj;
                traj_time := ClkRead(clk_traj);
                TPWrite "[MotionServer] Processing traj took ms: " + ValToStr(traj_time) + "/" + ValToStr(trajectory_size) + " = " + ValToStr((traj_time / trajectory_size) * 1000) + "ms per Point";
            ENDIF
            !SpyStop;
        CASE ROS_TRAJECTORY_STOP:
            is_not_receiving_traj := TRUE;
            IF(isDebug) THEN
                TPWrite "[MotionServer] Traj STOP received";
            ENDIF
            ! empty trajectory
            trajectory_size := 0;
            activate_trajectory;
            ! issue stop command just to be safe
            StopMove; ClearPath; StartMove;
        DEFAULT:
            add_traj_pt point, velocity;
    ENDTEST

    IF (message.header.comm_type = ROS_COM_TYPE_SRV_REQ) THEN
        IF(isDebug) THEN
            TPWrite "[MotionServer] Replying to service msg of type: " + ValToStr(message.header.msg_type);
        ENDIF
        reply_msg.header := [message.header.msg_type, ROS_COM_TYPE_SRV_REPLY, ROS_REPLY_TYPE_SUCCESS];
        ROS_send_msg client_socket, reply_msg;
    ENDIF

ERROR
    ! raise errors to calling code
    RAISE;
ENDPROC

LOCAL PROC add_traj_pt(jointtarget point, num velocity)
    IF (trajectory_size >= MAX_TRAJ_LENGTH) THEN
        ErrWrite \W, "[MotionServer] Too Many Trajectory Points", "Trajectory has already reached its maximum size",
                 \RL2:="max_size = " + ValToStr(MAX_TRAJ_LENGTH);
        TPWrite "[MotionServer] Received too many trajectory points. Limit is " + ValToStr(MAX_TRAJ_LENGTH);
        trajectory_size := 0;
        Stop;
    ELSE
        Incr trajectory_size;
        !add point to trajectory
        traj_buffer{trajectory_size} := point;
        velocities{trajectory_size} := velocity;
    ENDIF
ENDPROC

LOCAL PROC activate_trajectory()
    IF(isDebug) THEN
        TPWrite "[MotionServer] Sending " + ValToStr(trajectory_size) + " points to MOTION task";
    ENDIF
    WaitTestAndSet ROS_trajectory_lock;
    traj_size_pers := trajectory_size;
    ROS_new_trajectory := TRUE;
    ROS_trajectory_lock := FALSE;
ENDPROC

ENDMODULE
