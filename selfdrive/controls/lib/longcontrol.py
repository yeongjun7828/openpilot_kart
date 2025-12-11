from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.modeld.constants import T_IDXS

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  
  # Use threshold to avoid false positives from numerical noise when both speeds are very low
  # If both speeds are below 0.05 m/s (~0.18 km/h), require >0.01 m/s difference to be "accelerating"
  # Otherwise use normal comparison
  #print("v ego :", v_ego)
  #print("v_target :", v_target)
  #print("vtarget1sec-vtarget", (v_target_1sec - v_target))
 # cruise_standstill = True
#  if v_target < 0.05 and v_target_1sec < 0.05:
 #   accelerating = (v_target_1sec - v_target) > 0.01
 # else:
 #   accelerating = v_target_1sec > v_target
  threshold_speed = CP.vEgoStopping  # 0.15
# 가속으로 인정할 최소 차이 (현재 로그의 0.025 무시를 위해 0.03~0.05 정도로 상향)
  accel_threshold = 0.05

  if v_target < threshold_speed and v_target_1sec < threshold_speed:
  # 미세한 속도 증가는 가속으로 보지 않음 (노이즈 무시)
   accelerating = (v_target_1sec - v_target) > accel_threshold
  else:
   accelerating = v_target_1sec > v_target

  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = stay_stopped or planned_stop

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting
 # print("v_ego:", v_ego, " v_target:", v_target, " v_target_1sec:", v_target_1sec)
  print("accelerating:", accelerating, " planned_stop : ", planned_stop, " stay_stopped :", stay_stopped, " stopping_condition :", stopping_condition, " starting_condition :", starting_condition, " started_condition :", started_condition, " cruise_standstill :", cruise_standstill)
  if not active:
    long_control_state = LongCtrlState.off
    print("not active")
  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      print("In PID state")
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
        print("stopping")

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
    #print("starting_condition")
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
     #   print("started to pid")
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
#    print("active:", active)
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      # v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, T_IDXS[:CONTROL_N], speeds)
      v_target_1sec = interp(t_since_plan + 1.0, T_IDXS[:CONTROL_N], speeds)

    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    
 #   print("long_control_state:", self.long_control_state)
    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      print("In stopping state")
      #print("output_accel before clip:", output_accel)
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, -0.3)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      print("pid state")
      # print("v_pid:", self.v_pid, "v_ego:", CS.vEgo, "error:", error)
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      # print("output_accel (before clip):", output_accel)
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
