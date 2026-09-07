MODULE ROS_stateServer

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

LOCAL VAR intnum intr_rob_at_goal;
LOCAL CONST num server_port := 11002;
! target update rate in hz, if controller is not reaching desired update rate, reduce this value
CONST num STATE_HZ := 50;
VAR num state_update_rate := 1 / STATE_HZ;
! min update rate that is targeted during trajectory transmission
CONST num MIN_STATE_HZ := 3;
VAR num min_state_update_rate := 1.0 / MIN_STATE_HZ;
! send status with every Nth joint update
CONST num status_n := 35;
VAR bool robotAtGoalEvent := FALSE;
VAR MIKADO_connection_info connection_info;

LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;
LOCAL VAR jointtarget joints;
LOCAL VAR ROS_msg_dynamic_joints_data joint_message;
LOCAL VAR ROS_msg_robot_status status_message;

PROC main()
    ! Set up interrupt to watch for new trajectory
    ! TODO: check if sockets are initialized, if so they should maybe be reset so that they wait for a new connection properly
    VAR clock clk;
    VAR num next_t;
    VAR clock rate_clk;
    VAR num   rate_count := 0;
    VAR num i := 1;
    VAR bool  rate_clk_started := FALSE;
    ! clear interrupt handler, in case restarted with ExitCycle
    IDelete intr_rob_at_goal;
    CONNECT intr_rob_at_goal WITH robot_at_goal_handler;
    IPers robotAtGoal, intr_rob_at_goal;

    TPWrite "[StateServer] Waiting for connection.";
    ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket, "StateServer";
    ! wait for connection info message and configure driver
    MIKADO_receive_connection_info client_socket, connection_info;
    TPWrite "[StateServer] Received connection info. Num state axes: " + ValToStr(connection_info.num_ax_state);

    prepare_messages;

    IF(isDebug) THEN
        ClkStart rate_clk;
        rate_clk_started := TRUE;
    ENDIF

    WHILE TRUE DO
        send_joints;
        IF((i MOD status_n = 0) OR is_not_receiving_traj = FALSE)THEN
            send_status;
            i := 1;
        ELSE
            i := i +1;
        ENDIF

        IF(isDebug) THEN
            rate_count := rate_count + 1;
            IF ClkRead(rate_clk) >= 5.0 THEN
                TPWrite "[StateServer] State update rate: " +
                        NumToStr(rate_count / ClkRead(rate_clk), 3) +
                        " Hz (last " +
                        NumToStr(ClkRead(rate_clk), 2) +
                        " s)";
                rate_count := 0;
                ClkReset rate_clk;
                ClkStart rate_clk;
            ENDIF
        ENDIF

        IF(is_not_receiving_traj = FALSE) THEN
            InterruptibleWait(min_state_update_rate);
        ELSE
            InterruptibleWait(state_update_rate);
        ENDIF
    ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED)
    IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "[StateServer] ROS StateServer disconnect", "Connection lost.  Waiting for new connection.";
        ExitCycle;  ! restart program
    ELSE
        TRYNEXT;
    ENDIF
UNDO
ENDPROC

LOCAL PROC send_joints()
    ! get current joint position (degrees)
    joints := CJointT();

    ! create message
    joint_message.joints := joints.robax;
    IF(connection_info.num_eax_state>0) THEN
        joint_message.ext_axes := joints.extax;
    ENDIF
    IF(connection_info.is_jstate_is_moving) THEN
        IF DOutput(signalRobotNotMoving)=1 THEN
            joint_message.is_moving:=FALSE;
        ELSE
            joint_message.is_moving:=TRUE;
        ENDIF
    ENDIF

    ! send message to client
    ROS_send_msg_dynamic_joints_data client_socket, joint_message;

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

! signalExecutionError : System Output
! signalMotionPossible : System Output
! signalMotorOn : System Output
! signalRobotActive : System Output
! signalRobotEStop : System Output
! signalRobotNotMoving : System Output
! signalRosMotionTaskExecuting : System Output
LOCAL PROC send_status()
    ! Get operating mode
    TEST OpMode()
        CASE OP_AUTO:
            status_message.mode := ROS_ROBOT_MODE_AUTO;
        CASE OP_MAN_PROG, OP_MAN_TEST:
            status_message.mode := ROS_ROBOT_MODE_MANUAL;
        CASE OP_UNDEF:
            status_message.mode := ROS_ROBOT_MODE_UNKNOWN;
    ENDTEST

    ! Get E-stop status
    IF DOutput(signalRobotEStop) = 1 THEN
        status_message.e_stopped := ROS_TRISTATE_ON;
    ELSE
        status_message.e_stopped := ROS_TRISTATE_OFF;
    ENDIF

    ! Get whether motors have power
    IF DOutput(signalMotorOn) = 1 THEN
        status_message.drives_powered := ROS_TRISTATE_TRUE;
    ELSE
        status_message.drives_powered := ROS_TRISTATE_FALSE;
    ENDIF

    ! Determine in_error and set error_code if in_error is true
    if DOutput(signalExecutionError) = 1 THEN
        status_message.in_error := ROS_TRISTATE_TRUE;
        status_message.error_code := ERRNO;
    ELSE
        status_message.in_error := ROS_TRISTATE_FALSE;
        status_message.error_code := 0;
    ENDIF

    ! Get in_motion
    IF DOutput(signalRobotNotMoving) = 1 THEN
        status_message.in_motion := ROS_TRISTATE_FALSE;
    ELSE
        status_message.in_motion := ROS_TRISTATE_TRUE;
    ENDIF

    ! Get whether motion is possible
    if (DOutput(signalMotionPossible) = 1) AND
       (DOutput(signalRobotActive) = 1) AND
       (DOutput(signalMotorOn) = 1) THEN
        status_message.motion_possible := ROS_TRISTATE_TRUE;
    ELSE
        status_message.motion_possible := ROS_TRISTATE_FALSE;
    ENDIF

    ! send status_message to client
    ROS_send_msg_robot_status client_socket, status_message;

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

LOCAL TRAP robot_at_goal_handler
    IF (NOT robotAtGoal) RETURN;
    robotAtGoalEvent := TRUE;
ENDTRAP

PROC InterruptibleWait(num totalTime)
    VAR num t := 0;
    WHILE t < totalTime DO
        IF robotAtGoalEvent THEN
            robotAtGoalEvent := FALSE;
            send_joints;
            ! uncomment send_status if not sending is_moving in joint update
            !send_status;
        ENDIF
        WaitTime 0.01;
        t := t + 0.01;
    ENDWHILE
ENDPROC

LOCAL PROC prepare_messages()
    ! prepare joint message
    joint_message.header := [ROS_MSG_TYPE_JOINT, ROS_COM_TYPE_TOPIC, ROS_REPLY_TYPE_INVALID];
    joint_message.sequence_id := 0;

    ! prepare status message
    status_message.header := [ROS_MSG_TYPE_STATUS, ROS_COM_TYPE_TOPIC, ROS_REPLY_TYPE_INVALID];
    status_message.sequence_id := 0;

    ! default values
    status_message.mode            := ROS_ROBOT_MODE_UNKNOWN;
    status_message.e_stopped       := ROS_TRISTATE_UNKNOWN;
    status_message.drives_powered  := ROS_TRISTATE_UNKNOWN;
    status_message.error_code      := ROS_TRISTATE_UNKNOWN;
    status_message.in_error        := ROS_TRISTATE_UNKNOWN;
    status_message.in_motion       := ROS_TRISTATE_UNKNOWN;
    status_message.motion_possible := ROS_TRISTATE_UNKNOWN;
ENDPROC

ENDMODULE
