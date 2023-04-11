#!/usr/bin/env python3
import os
import json
from atomicwrites import atomic_write
from common.colors import COLORS
from common.travis_checker import BASEDIR
from selfdrive.hardware import TICI
from common.params import Params
from common.numpy_fast import clip

try:
  from common.realtime import sec_since_boot
except ImportError:
  import time
  sec_since_boot = time.time

warning = lambda msg: print('{}opParams WARNING: {}{}'.format(COLORS.WARNING, msg, COLORS.ENDC))
error = lambda msg: print('{}opParams ERROR: {}{}'.format(COLORS.FAIL, msg, COLORS.ENDC))

NUMBER = [float, int]  # value types
NONE_OR_NUMBER = [type(None), float, int]

BASEDIR = os.path.dirname(BASEDIR)
PARAMS_DIR = os.path.join(BASEDIR, 'community', 'params')
IMPORTED_PATH = os.path.join(PARAMS_DIR, '.imported')
OLD_PARAMS_FILE = os.path.join(BASEDIR, 'op_params.json')


class Param:
  def __init__(self, 
               default, 
               allowed_types=[], 
               description=None, 
               *, 
               static=False, 
               live=False, 
               hidden=False,
               allowed_vals=[],
               min_val=None,
               max_val=None,
               is_common=False,
               param_param='',
               param_param_use_ord=False,
               unit='',
               linked_op_param_check_param='',
               param_param_read_on_startup=False,
               show_op_param='',
               show_op_param_check_val=None,
               fake_live=False):  # pylint: disable=dangerous-default-value
    self.default_value = default  # value first saved and returned if actual value isn't a valid type
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types  # allowed python value types for opEdit
    self.description = description  # description to be shown in opEdit
    self.hidden = hidden  # hide this param to user in opEdit
    self.live = live  # show under the live menu in opEdit
    self.static = static  # use cached value, never reads to update
    self.fake_live = fake_live # param is live but only read on OP startup
    self.allowed_vals = allowed_vals # optional to specify a set of discrete allowed values (of any type)
    self.min_val = min_val # specify minimum value
    self.max_val = max_val # specify maximum value
    self.is_common = is_common
    self.unit = unit
    self.show_op_param = show_op_param # if set, this param's value is compared to see if param should be shown or not
    self.show_op_param_check_val = show_op_param_check_val # the value against which it is compared
    self.linked_op_param_check_param = linked_op_param_check_param # this specified bool param can be changed by the user to control whether the list items are synced
    self.param_param = param_param # op_params can also write to regular "params" when put(), and op_params always overwrite regular params
    self.param_param_use_ord = param_param_use_ord # store the index of the value in allowed values when writing to corresponding param
    self.param_param_read_on_startup = param_param_read_on_startup # if true, get the param_param when initializing to override own value
    self._get_thread = None # non-static params are fetched regularly in a separate thread
    self._create_attrs()

  def type_is_valid(self, value):
    if not self.has_allowed_types:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return all([not self.is_list,
                type(value) in self.allowed_types]) \
      or all([self.is_list,
              isinstance(value, list) and all([type(i) in self.allowed_types for i in value])])
  
  def value_is_valid(self, value):
    if not self.has_allowed_vals:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return value in self.allowed_vals
  
  def is_valid(self, value):
    if not self.has_allowed_types:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return type(value) in self.allowed_types

  def clip_val(self, v):
    if self.max_val is not None and self.min_val is not None:
      return clip(v, self.min_val, self.max_val)
    elif self.max_val is not None:
      return min(self.max_val, v)
    elif self.min_val is not None:
      return max(self.min_val, v)
    return v
    
  def value_clipped(self, value):
    
    if isinstance(value, list):
      val = [self.clip_val(v) for v in value]
    else:
      val = self.clip_val(value)
    
    if val != value:
      value = val
      return value, True
    else:
      return value, False
  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_allowed_vals = isinstance(self.allowed_vals, list) and len(self.allowed_vals) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    self.read_frequency = None if self.static else (1 if self.live else 10)  # how often to read param file (sec)
    if self.has_allowed_types:
      if not type(self.default_value) in self.allowed_types and not isinstance(self.default_value, list):
        try:
          self.default_value = self.allowed_types[0](self.default_value)
        except ValueError:
          assert type(self.default_value) in self.allowed_types, 'Default value type must be in specified allowed_types!'
    if self.has_allowed_vals:
      assert self.default_value in self.allowed_vals, 'Default value must be in specified allowed_vals!'
    self.default_value = self.value_clipped(self.default_value)[0]
    self.updated = False
    self.last_read = 0
    self._params = None
    self.value = None
    if self.param_param != '':
      self._params = Params()
      try:
        val = None
        self._params.check_key(self.param_param)
        if self.param_param_read_on_startup:
          if self.param_param_use_ord and self.has_allowed_vals:
            val = int(self._params.get(self.param_param, encoding="utf8"))
            if val < len(self.allowed_vals):
              self.value = self.allowed_vals[val]
            else:
              self.value = self._params.get(self.param_param)
          else:
            self.value = self._params.get(self.param_param)
          if self.has_allowed_types and type(self.value) not in self.allowed_types:
            for t in self.allowed_types:
              try:
                self.value = t(self.value)
                break
              except:
                continue
            if type(self.value) not in self.allowed_types:
              raise ValueError 
          # cloudlog.info(f"opParams: Read in param value '{self.value}' from param '{self.param_param}' on startup")
        else:
          self.value = self.default_value
      except Exception as e:
        self.param_param = ''
        self._params = None
        self.value = self.default_value
    else:
      self.value = self.default_value


def _read_param(key):  # Returns None, False if a json error occurs
  try:
    with open(os.path.join(PARAMS_DIR, key), 'r') as f:
      value = json.loads(f.read())
    return value, True
  except json.decoder.JSONDecodeError:
    return None, False


def _write_param(key, value, reason=None, old_value=None, do_log=True):
  
  param_path = os.path.join(PARAMS_DIR, key)
  with atomic_write(param_path, overwrite=True) as f:
    f.write(json.dumps(value))


def _import_params():
  if os.path.exists(OLD_PARAMS_FILE) and not os.path.exists(IMPORTED_PATH):  # if opParams needs to import from old params file
    try:
      with open(OLD_PARAMS_FILE, 'r') as f:
        old_params = json.loads(f.read())
      for key in old_params:
        (key, old_params[key])
      open(IMPORTED_PATH, 'w').close()
    except:  # pylint: disable=bare-except
      pass


class opParams:
  def __init__(self, calling_function=''):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
          (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - If you want your param to update within a second, specify live=True. If your param is designed to be read once, specify static=True.
          Specifying neither will have the param update every 10 seconds if constantly .get()
          If the param is not static, call the .get() function on it in the update function of the file you're reading from to use live updating

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(0.06, allowed_types=NUMBER), live=True}  # NUMBER allows both floats and ints
    """

    kf_desc = 'Feedforward is the part of the steering controller that only cares about the desire steering angle (how sharp the curve is). So feedforward only comes into play in curves when the desired steering angle is non-zero, and the greater the angle, the greater the feedforward response, which is scaled by kf.\nTo tune kf, you observe if OpenPilot enters curves too early/late and rides curves too far inside/outside. If it enters too early (late) and/or rides too far inside (outside), then kf is too high (low) and should be lowered (raised) in 10% increments until it enters correctly and rides center.\n'
    kp_desc = 'Proportional gain responds proportionally to the instantaneous error being controlled. The greater the error, the greater the corrective response, linearly, and scaled according to kp. In this case, where we\'re controlling the steering angle, the proportional gain alone cannot completely correct for error, becuase when the error is close to zero, so is the proportional response.\nThe best way to tune kp is then using nudgeless lane change on straight roads (no feedforward response), which creates a sudden (so doesn\'t trigger the integral response) change in course that results in a reproducible error source that triggers the proportional and derivative responses. Set kd to zero to best assess kp. If the lane change feels too assertive or jerky, lower kp. If too weak, increase kp.\n'
    ki_desc = 'Integral gain responds based on the accumulated error, so if you\'re missing the target continually, the integral response builds the longer you\'re off in the same direction. This corrects for things like persistent crosswinds, inconsistent tire pressures, or dramatic road roll that roll compensation fails to fully compensate for. The drawback is that integral gain can "wind up", overshooting the desired angle, causing lateral oscillations, ping-ponging back and forth about lane center.\nTune kf and kp with ki set to zero, then set ki to 1/3rd the value of kp or less, and kd to 10-20x the value of kp (see the default values here). If lateral oscillations occur, lower ki in 10% increments until they are no longer observed.\n'
    kd_desc = 'Derivative gain responds to the rate of change of error. The benefits are two-fold. First, note that the proportional and integral responses always push against the error until the error is zero (and due to integral wind-up, integral can push past zero even), which necessarily results in overshoot and oscillations. In such an overshooting case, when you\'re returning to lane center and the error (let\'s say positive) is decreasing, the error rate wil be negative even though the error is still positive, so the derivative response is pushing against the proportional and integral overshoot. Second, if you\'re quickly leaving lane center then the rate of change of error is positive along with the error, so the derivative here helps along with the proportional and integral responses to correct for the error. Too high of kd is indicated by a jerky initial correction when using nudgeles lane change on straight roads.\n'
   
    self.fork_params = {#'camera_offset': Param(-0.04 if TICI else 0.06, NUMBER, 'Your camera offset to use in lane_planner.py\n'
                        #                                                        'If you have a comma three, note that the default camera offset is -0.04!', live=True),
                        #'dynamic_follow': Param('stock', str, static=True, hidden=True),
                        #'global_df_mod': Param(1.0, NUMBER, 'The multiplier for the current distance used by dynamic follow. The range is limited from 0.85 to 2.5\n'
                        #                                    'Smaller values will get you closer, larger will get you farther\n'
                        #                                    'This is multiplied by any profile that\'s active. Set to 1. to disable', live=True),
                        #'min_TR': Param(0.9, NUMBER, 'The minimum allowed following distance in seconds. Default is 0.9 seconds\n'
                        #                             'The range is limited from 0.85 to 2.7', live=True),
                        #'alca_no_nudge_speed': Param(90., NUMBER, 'Above this speed (mph), lane changes initiate IMMEDIATELY. Behavior is stock under'),
                        #'steer_ratio': Param(None, NONE_OR_NUMBER, '(Can be: None, or a float) If you enter None, openpilot will use the learned sR.\n'
                        #                                           'If you use a float/int, openpilot will use that steer ratio instead', live=True),
                        # 'lane_speed_alerts': Param('silent', str, 'Can be: (\'off\', \'silent\', \'audible\')\n'
                        #                                           'Whether you want openpilot to alert you of faster-traveling adjacent lanes'),
                        #'upload_on_hotspot': Param(False, bool, 'If False, openpilot will not upload driving data while connected to your phone\'s hotspot'),
                        #'disengage_on_gas': Param(False, bool, 'Whether you want openpilot to disengage on gas input or not'),
                        #'update_behavior': Param('alert', str, 'Can be: (\'off\', \'alert\', \'auto\') without quotes\n'
                        #                                       'off will never update, alert shows an alert on-screen\n'
                        #                                       'auto will reboot the device when an update is seen', static=True),
                        #'dynamic_gas': Param(False, bool, 'Whether to use dynamic gas if your car is supported'),
                        #'hide_auto_df_alerts': Param(False, bool, 'Hides the alert that shows what profile the model has chosen'),
                        #'df_button_alerts': Param('audible', str, 'Can be: (\'off\', \'silent\', \'audible\')\n'
                        #                                          'How you want to be alerted when you change your dynamic following profile.'),
                        # 'log_auto_df': Param(False, bool, 'Logs dynamic follow data for auto-df', static=True),
                        # 'lat_p': Param(0.2, NUMBER, 'Actual proportional gain. Start from your car\'s tune in interface.py', live=True),
                        # 'lat_i': Param(0.05, NUMBER, 'Actual integral gain. Start from your car\'s tune in interface.py', live=True),
                        # 'lat_d': Param(0.0, NUMBER, 'Actual derivative gain. Does not exist in stock openpilot', live=True),
                        # 'lat_f_multiplier': Param(1., NUMBER, 'Multiplier for your existing feedforward gain. 1 is default', live=True),
                        'TUNE_LAT_do_override': Param(False, bool, 'If true, the other params here will override the hardcoded lateral tune settings for any gm car. Changes to this opParam will also apply to the "Custom lateral override" toggle in OpenPilot settings.\n', param_param='OPParamsLateralOverride', fake_live=True, param_param_read_on_startup=True),
                        'TUNE_LAT_PID_kf': Param(0.00008, float, kf_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
                        'TUNE_LAT_PID_kp': Param([0.05, 0.10, 0.15, 0.16], [list, float], kp_desc + 'This scales the low-speed response.\n', live=True, min_val=0.0, max_val=10.0, linked_op_param_check_param='TUNE_LAT_PID_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
                        'TUNE_LAT_PID_ki': Param([0.08, 0.3, 0.3, 0.45], [list, float], ki_desc + 'This scales the low-speed response.\n', live=True, min_val=0.0, max_val=10.0,  linked_op_param_check_param='TUNE_LAT_PID_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
                        'TUNE_LAT_PID_bp_ms': Param([0.0, 10., 20., 30.], [list, float], 'These speeds corresponds to the low- and high-speed kp, ki, and kd values.\n', live=True, min_val=0.0, max_val=100.0, unit="ms", show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),

                        # 'dynamic_camera_offset': Param(False, bool, 'Whether to automatically keep away from oncoming traffic.\n'
                        #                                             'Works from 35 to ~60 mph (requires radar)'),
                        # 'dynamic_camera_offset_time': Param(3.5, NUMBER, 'How long to keep away from oncoming traffic in seconds after losing lead'),
                        #'support_white_panda': Param(False, bool, 'Enable this to allow engagement with the deprecated white panda.\n'
                        #                                          'localizer might not work correctly', static=True),
                        #'disable_charging': Param(30, NUMBER, 'How many hours until charging is disabled while idle', static=True),
                        #'hide_model_long': Param(False, bool, 'Enable this to hide the Model Long button on the screen', static=True),
                        #'prius_use_pid': Param(False, bool, 'This enables the PID lateral controller with new a experimental derivative tune\n'
                        #                                    'False: stock INDI, True: TSS2-tuned PID', static=True),
                        #'use_lqr': Param(False, bool, 'Enable this to use LQR as your lateral controller over default with any car', static=True),
                        #'use_steering_model': Param(False, bool, 'Enable this to use an experimental ML-based lateral controller trained on the TSSP Corolla\n'
                        #                                         'This overrides all other tuning parameters\n'
                        #                                         'Warning: the model may behave unexpectedly at any time, so always pay attention', static=True),
                        #'corollaTSS2_use_indi': Param(False, bool, 'Enable this to use INDI for lat with your TSS2 Corolla', static=True),
                        #'rav4TSS2_use_indi': Param(False, bool, 'Enable this to use INDI for lat with your TSS2 RAV4', static=True),
                        #'standstill_hack': Param(False, bool, 'Some cars support stop and go, you just need to enable this', static=True)
                        }

    self._to_delete = ['enable_long_derivative']  # a list of unused params you want to delete from users' params file
    self._to_reset = []  # a list of params you want reset to their default values
    self._run_init()  # restores, reads, and updates params

  def _run_init(self):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.fork_params['username'] = Param(None, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)

    self.params = self._load_params(can_import=True)
    self._add_default_params()  # adds missing params and resets values with invalid types to self.params
    self._delete_and_reset()  # removes old params

  def get(self, key=None, *, force_update=False):  # key=None returns dict of all params
    if key is None:
      return self._get_all_params(to_update=force_update)
    self._check_key_exists(key, 'get')
    param_info = self.fork_params[key]
    rate = param_info.read_frequency  # will be None if param is static, so check below

    if (not param_info.static and sec_since_boot() - self.fork_params[key].last_read >= rate) or force_update:
      value, success = _read_param(key)
      self.fork_params[key].last_read = sec_since_boot()
      if not success:  # in case of read error, use default and overwrite param
        value = param_info.default_value
        _write_param(key, value)
      self.params[key] = value

    if param_info.is_valid(value := self.params[key]):
      return value  # all good, returning user's value
    print(warning('User\'s value type is not valid! Returning default'))  # somehow... it should always be valid
    return param_info.default_value  # return default value because user's value of key is not in allowed_types to avoid crashing openpilot

  def put(self, key, value, reason="changed by user", show_alert=False, do_log=True):
    self._check_key_exists(key, 'put')
    old_val = self.fork_params[key].value
    if not self.fork_params[key].type_is_valid(value):
      raise Exception(f'opParams: Tried to put a value of invalid type! {key = }, {value = }')
    if not self.fork_params[key].value_is_valid(value):
      raise Exception(f'opParams: Tried to put an invalid value! {key = }, {value = }')
    value, clipped = self.fork_params[key].value_clipped(value)
    if clipped:
      print(warning(f'Provided value was clipped to param bounds. {key = }, {value = }'))
    self.params.update({key: value})
    self.fork_params[key].value = value
    if self.fork_params[key].param_param != '':
      if self.fork_params[key].param_param_use_ord and value in self.fork_params[key].allowed_vals:
        ind = self.fork_params[key].allowed_vals.index(value)
        self.fork_params[key]._params.put(self.fork_params[key].param_param, str(ind))
      else:
        put_val = value if type(value) == str \
          else str(int(value)) if type(value) == bool \
          else str(value)
        self.fork_params[key]._params.put(self.fork_params[key].param_param, put_val)
    _write_param(key, value, reason=reason, old_value=old_val, do_log=do_log)
    if show_alert:
      _write_param('op_edit_param_changed', True, reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_name', key, reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_val_old', str(old_val), reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_val_new', str(value), reason=reason, old_value=old_val, do_log=False)

  def _load_params(self, can_import=False):
    if not os.path.exists(PARAMS_DIR):
      os.makedirs(PARAMS_DIR)
      if can_import:
        _import_params()  # just imports old params. below we read them in

    params = {}
    for key in os.listdir(PARAMS_DIR):  # PARAMS_DIR is guaranteed to exist
      if key.startswith('.') or key not in self.fork_params:
        continue
      value, success = _read_param(key)
      if not success:
        value = self.fork_params[key].default_value
        _write_param(key, value)
      params[key] = value
    return params

  def _get_all_params(self, to_update=False):
    if to_update:
      self.params = self._load_params()
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and not p.hidden}

  def _check_key_exists(self, key, met):
    if key not in self.fork_params:
      raise Exception('opParams: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(met, key))

  def _add_default_params(self):
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default_value
        _write_param(key, self.params[key])
      elif not param.is_valid(self.params[key]):
        print(warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key)))
        self.params[key] = param.default_value
        _write_param(key, self.params[key])

  def _delete_and_reset(self):
    for key in list(self.params):
      if key in self._to_delete:
        del self.params[key]
        os.remove(os.path.join(PARAMS_DIR, key))
      elif key in self._to_reset and key in self.fork_params:
        self.params[key] = self.fork_params[key].default_value
        _write_param(key, self.params[key])
