#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import custom
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0

def get_jerk_factor(personality=custom.LongitudinalPersonalitySP.standard):
  if personality==custom.LongitudinalPersonalitySP.relaxed:
    return 1.3
  elif personality==custom.LongitudinalPersonalitySP.standard:
    return 1.0
  elif personality==custom.LongitudinalPersonalitySP.moderate:
    return 0.5
  elif personality==custom.LongitudinalPersonalitySP.aggressive:
    return 0.2
  elif personality==custom.LongitudinalPersonalitySP.overtake:
    return 0.1
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_a_change_factor(v_ego, v_lead0, v_lead1, personality=custom.LongitudinalPersonalitySP.standard):
  # Set cost multipliers based on driving personality (relaxed, standard, moderate, aggressive).
  # These values adjust the sensitivity of acceleration change.
  # Higher value = more cautious (slower reaction), smaller value = quicker response (more aggressive driving)
  if personality==custom.LongitudinalPersonalitySP.relaxed:
    a_change_cost_multiplier_follow = 1.0  # Highest cost for changing acceleration, meaning more gradual transitions
    a_change_cost_high_speed_factor = 1.0  # No extra penalty for high-speed changes (more cautious)
  elif personality==custom.LongitudinalPersonalitySP.standard:
    a_change_cost_multiplier_follow = 0.5  # Moderate cost for changing acceleration (quicker transitions compared to relaxed)
    a_change_cost_high_speed_factor = 1.5  # Higher penalty for changes at higher speeds (more cautious)
  elif personality==custom.LongitudinalPersonalitySP.moderate:
    a_change_cost_multiplier_follow = 0.5  # Similar to standard (quicker transitions compared to relaxed)
    a_change_cost_high_speed_factor = 1.5  # Similar to standard (higher penalty for high speeds)
  elif personality==custom.LongitudinalPersonalitySP.aggressive:
    a_change_cost_multiplier_follow = 0.1  # Very low cost for changing acceleration, meaning quicker reactions (less cautious)
    a_change_cost_high_speed_factor = 3.0  # Much higher penalty for abrupt changes at high speeds (very cautious at high speeds)
  elif personality==custom.LongitudinalPersonalitySP.overtake:
    a_change_cost_multiplier_follow = 0.1  # Very low cost for changing acceleration, meaning quicker reactions (less cautious)
    a_change_cost_high_speed_factor = 5.0  # Much higher penalty for abrupt changes at high speeds (very cautious at high speeds)
  else:
    raise NotImplementedError("Longitudinal personality not supported")

  # Variables to modify the acceleration change based on speed and lead vehicle conditions.
  # LEAD_AUGMENTATION_BP_MAX defines the vEgo threshold for rapid acceleration.
  LEAD_AUGMENTATION_BP_MAX = 5.0  # Maximum speed (5 m/s ~ 18 km/h) where rapid acceleration adjustments are allowed

  # LEAD_AUGMENTATION_BP: breakpoints for ego vehicle speed (vEgo) in m/s
  # LEAD_AUGMENTATION_V: multiplier values for ego vehicle speed interpolation
  LEAD_AUGMENTATION_BP = [0., LEAD_AUGMENTATION_BP_MAX]  # vEgo breakpoints: [0 m/s, 5 m/s (~18 km/h)]
  LEAD_AUGMENTATION_V = [.05, 1.]  # acceleration multipliers: At 0 m/s, allow very small changes (.05), at 5 m/s allow faster acceleration (1.0)

  # SPEED_AUGMENTATION_BP: breakpoints for speed adjustment to reduce abrupt braking at higher speeds
  # SPEED_AUGMENTATION_V: interpolation values for scaling acceleration cost based on speed
  # Higher = more cautious (penalizes abrupt braking), smaller = more aggressive (less penalty)
  SPEED_AUGMENTATION_BP = [0., LEAD_AUGMENTATION_BP_MAX, 10.]  # Speed breakpoints: [0 m/s, 5 m/s, 10 m/s (~36 km/h)]
  SPEED_AUGMENTATION_V = [1., 1., a_change_cost_high_speed_factor]  # Multiplier: between 0-5 m/s, no change (1.0), after 5 m/s, scale by a_change_cost_high_speed_factor (e.g., 1.5 in standard mode)

  # Calculate a cost for acceleration changes when lead vehicles are pulling away and ego speed is below the threshold.
  lead_augmented_a_change_cost = 1.0  # Default cost factor
  if (v_lead0 - v_ego > 1e-3) and (v_lead1 - v_ego > 1e-3):
    # Interpolate for the acceleration change cost when lead vehicles are increasing speed, based on vEgo
    lead_augmented_a_change_cost = interp(v_ego, LEAD_AUGMENTATION_BP, LEAD_AUGMENTATION_V)

  # Multiply the lead-based cost with speed-based cost to get a final cost factor, scaling with vehicle speed
  speed_augmented_a_change_cost = a_change_cost_multiplier_follow * interp(v_ego, SPEED_AUGMENTATION_BP, SPEED_AUGMENTATION_V)

  # Choose the smaller factor between the lead-based cost and the speed-based cost
  a_change_factor = lead_augmented_a_change_cost if v_ego <= LEAD_AUGMENTATION_BP_MAX else speed_augmented_a_change_cost

  # Return the final acceleration change factor to be applied
  return a_change_factor

# Function to return a multiplier for a danger zone cost based on driving personality
def get_danger_zone_factor(personality=custom.LongitudinalPersonalitySP.standard):
  # Higher values mean more cautious driving in dangerous situations, scaling the cost accordingly
  if personality==custom.LongitudinalPersonalitySP.relaxed:
    return 1.6  # Higher danger zone cost for relaxed personality (more cautious)
  elif personality==custom.LongitudinalPersonalitySP.standard:
    return 1.3  # Medium danger zone cost for standard personality
  elif personality==custom.LongitudinalPersonalitySP.moderate:
    return 1.3  # Medium danger zone cost for moderate personality (similar to standard)
  elif personality==custom.LongitudinalPersonalitySP.aggressive:
    return 1.0  # Lowest danger zone cost for aggressive personality (less cautious)
  elif personality==custom.LongitudinalPersonalitySP.overtake:
    return 1.0  # Lowest danger zone cost for aggressive personality (less cautious)
  else:
    raise NotImplementedError("Longitudinal personality not supported")



def get_T_FOLLOW(personality=custom.LongitudinalPersonalitySP.standard):
  if personality==custom.LongitudinalPersonalitySP.relaxed:
    return 1.75
  elif personality==custom.LongitudinalPersonalitySP.standard:
    return 1.65
  elif personality==custom.LongitudinalPersonalitySP.moderate:
    return 1.45
  elif personality==custom.LongitudinalPersonalitySP.aggressive:
    return 1.25
  elif personality==custom.LongitudinalPersonalitySP.overtake:
    return 0.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_dynamic_personality(v_ego, personality=custom.LongitudinalPersonalitySP.standard):
  if personality==custom.LongitudinalPersonalitySP.relaxed:
    x_vel =  [0.,   22.,   22.01,  36.1]
    y_dist = [1.70, 1.70,  1.80,   1.80]
  elif personality==custom.LongitudinalPersonalitySP.standard:
    x_vel =  [0.,   22.,   22.01,  36.1]
    y_dist = [1.65, 1.65,  1.75,   1.75]
  elif personality==custom.LongitudinalPersonalitySP.moderate:
    x_vel =  [0.,   22.,   22.01,  36.1]
    y_dist = [1.45, 1.45,  1.55,   1.55]
  elif personality==custom.LongitudinalPersonalitySP.aggressive:
    x_vel =  [0.,   19.7,  19.71,  36.1]
    y_dist = [1.25, 1.25,  1.35,   1.35]
  else:
    raise NotImplementedError("Dynamic personality not supported")
  return np.interp(v_ego, x_vel, y_dist)

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self.mode = mode
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

    self.e2e_x = np.zeros(13, dtype=np.float64)

  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, v_lead0 = 0., v_lead1 = 0., personality=custom.LongitudinalPersonalitySP.standard):
    v_ego = self.x0[1]
    jerk_factor = get_jerk_factor(personality)
    a_change_factor = get_a_change_factor(v_ego, v_lead0, v_lead1, personality)
    danger_zone_factor = get_danger_zone_factor(personality)
    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_factor * a_change_cost, jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST * danger_zone_factor]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, radarstate, v_cruise, prev_accel_constraint, x, v, a, j, personality=custom.LongitudinalPersonalitySP.standard,
             dynamic_personality=False, overtaking_acceleration_assist=False):
    v_ego = self.x0[1]
    t_follow = get_dynamic_personality(v_ego, personality) if dynamic_personality else get_T_FOLLOW(personality)
    t_follow = get_T_FOLLOW(custom.LongitudinalPersonalitySP.overtake) if overtaking_acceleration_assist else t_follow
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=lead_xv_0[0, 1], v_lead1=lead_xv_1[0, 1], personality=personality)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])

    cruise_target_e2ex = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
    e2e_xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
    e2e_x = np.cumsum(np.insert(e2e_xforward, 0, x[0]))

    x_and_cruise_e2ex = np.column_stack([e2e_x, cruise_target_e2ex])
    e2e_x = np.min(x_and_cruise_e2ex, axis=1)

    self.params[:,0] = ACCEL_MIN
    self.params[:,1] = self.max_a

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.e2e_x = e2e_x[:]

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow))- self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow))- self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    # t0 = time.monotonic()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, \
    # integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(time.monotonic() - t0):.2e} qp {self.time_qp_solution:.2e}, \
    # lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
