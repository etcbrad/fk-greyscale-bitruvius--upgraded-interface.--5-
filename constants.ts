

import { PartName, Pose, Vector2D, JointLimits } from './types';

export const SCALE_FACTOR = 3.5; // Doubled from 1.75

export const HEAD_UNIT = 50 * SCALE_FACTOR;

export const ANATOMY = {
  HEAD: 0.8 * HEAD_UNIT,
  HEAD_WIDTH: 0.6 * HEAD_UNIT,
  HEAD_NECK_GAP_OFFSET: 0.1 * HEAD_UNIT,
  COLLAR: 0.4 * HEAD_UNIT, 
  COLLAR_WIDTH: 0.3 * HEAD_UNIT, 
  TORSO: 1.4 * HEAD_UNIT,
  TORSO_WIDTH: 0.9 * HEAD_UNIT, 
  WAIST: 1.0 * HEAD_UNIT,
  WAIST_WIDTH: 0.8 * HEAD_UNIT,
  UPPER_ARM: 1.6 * HEAD_UNIT,
  LOWER_ARM: 1.3 * HEAD_UNIT,
  HAND: 0.7 * HEAD_UNIT,
  LEG_UPPER: 2.2 * HEAD_UNIT,
  LEG_LOWER: 2.0 * HEAD_UNIT,
  FOOT: 0.5 * HEAD_UNIT,
  SHOULDER_WIDTH: 0.6 * HEAD_UNIT,
  HIP_WIDTH: 0.5 * HEAD_UNIT,
  ROOT_SIZE: 0.15 * HEAD_UNIT,
  LIMB_WIDTH_ARM: 0.24 * HEAD_UNIT,
  LIMB_WIDTH_FOREARM: 0.18 * HEAD_UNIT,
  LIMB_WIDTH_THIGH: 0.18 * HEAD_UNIT,
  LIMB_WIDTH_CALF: 0.15 * HEAD_UNIT,
  HAND_WIDTH: 0.12 * HEAD_UNIT,
  FOOT_WIDTH: 0.15 * HEAD_UNIT,
  EFFECTOR_WIDTH: 0.06 * HEAD_UNIT,
  TOE: 0.3 * HEAD_UNIT,
};

export const RIGGING = {
  L_SHOULDER_X_OFFSET_FROM_COLLAR_CENTER: -ANATOMY.COLLAR_WIDTH / 2.1,
  R_SHOULDER_X_OFFSET_FROM_COLLAR_CENTER: ANATOMY.COLLAR_WIDTH / 2.1,
  /**
   * Lowering the shoulders by one "anchor length" (the height of the collar segment).
   * Since the collar draws upwards from chest to neck, an offset of ANATOMY.COLLAR 
   * from the end (neck) places the shoulders back at the chest/torso junction.
   */
  SHOULDER_Y_OFFSET_FROM_COLLAR_END: ANATOMY.COLLAR,
  COLLAR_OFFSET_Y: 0,
};

export const FLOOR_HEIGHT = 1000 * SCALE_FACTOR;
export const FLOOR_SINK_BUFFER = 50 * SCALE_FACTOR; // How much below floor for "slow ease" effect
export const GROUND_SINK_REMAINING_PROPORTION = 0.45; // Proportion of the sink depth that remains when in buffer zone

export const GROUND_STRIP_HEIGHT = 20 * SCALE_FACTOR; // Height of the visual ground strip
export const GROUND_STRIP_COLOR = '#252525'; // Slightly lighter very dark grey for the ground strip


export const T_POSE_ROOT_Y = FLOOR_HEIGHT - (ANATOMY.LEG_UPPER + ANATOMY.LEG_LOWER + ANATOMY.FOOT + ANATOMY.WAIST);

type RotationValues = Omit<Pose, 'root' | 'offsets' | 'vOffset'>;

export const BASE_ROTATIONS: RotationValues = {
  bodyRotation: 0,
  torso: 0,
  waist: 0,
  collar: 0,
  head: 0,
  lShoulder: 84.17,
  lForearm: 0,
  lWrist: 0,
  rShoulder: -84.98,
  rForearm: 0,
  rWrist: 0,
  lThigh: -0.02,
  lCalf: 0,
  lAnkle: 0,
  rThigh: 26.64,
  rCalf: 0,
  rAnkle: 0,
  lShoulderGirdle: 0,
  rShoulderGirdle: 0,
  lHipGirdle: 0,
  rHipGirdle: 0,
  lToe: 0,
  rToe: 0,
};

export const RESET_POSE: Pose = {
  root: { x: 0, y: T_POSE_ROOT_Y },
  bodyRotation: 0,
  torso: 0,
  waist: 0,
  collar: 0,
  head: 0,
  lShoulder: 0,
  lForearm: 0,
  lWrist: 0,
  rShoulder: 0,
  rForearm: 0,
  rWrist: 0,
  lThigh: 0,
  lCalf: 0,
  lAnkle: 0,
  rThigh: 0,
  rCalf: 0,
  rAnkle: 0,
  lShoulderGirdle: 0,
  rShoulderGirdle: 0,
  lHipGirdle: 0,
  rHipGirdle: 0,
  lToe: 0,
  rToe: 0,
  vOffset: { x: 0, y: 0 },
  offsets: {},
};

// Define joint rotation limits in degrees
export const JOINT_LIMITS: JointLimits = {
  // Spine (relative to parent)
  [PartName.Waist]: { min: -180, max: 180 }, 
  [PartName.Torso]: { min: -180, max: 180 },
  [PartName.Collar]: { min: -180, max: 180 },
  [PartName.Head]: { min: -55, max: 55 },

  // Right Arm (relative to parent)
  [PartName.RShoulder]: { min: -180, max: 180 }, 
  rForearm: { min: -180, max: 180 },         
  [PartName.RWrist]: { min: -180, max: 180 }, 

  // Left Arm (relative to parent)
  [PartName.LShoulder]: { min: -180, max: 180 }, 
  lForearm: { min: -180, max: 180 },          
  [PartName.LWrist]: { min: -180, max: 180 }, 

  // Right Leg (relative to parent)
  [PartName.RThigh]: { min: -180, max: 180 }, 
  rCalf: { min: -180, max: 180 },           
  [PartName.RAnkle]: { min: -180, max: 180 }, 
  // Left Leg (relative to parent)
  [PartName.LThigh]: { min: -180, max: 180 },
  lCalf: { min: -180, max: 180 },
  [PartName.LAnkle]: { min: -180, max: 180 },
  [PartName.LShoulderGirdle]: { min: -18, max: 18 },
  [PartName.RShoulderGirdle]: { min: -18, max: 18 },
  [PartName.LHipGirdle]: { min: -5, max: 5 },
  [PartName.RHipGirdle]: { min: -5, max: 5 },
  [PartName.LToe]: { min: -180, max: 180 },
  [PartName.RToe]: { min: -180, max: 180 },
};
