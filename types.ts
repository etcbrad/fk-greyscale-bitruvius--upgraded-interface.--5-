

import React from 'react';

export enum PartName {
  Torso = 'torso',
  Waist = 'waist',
  Collar = 'collar',
  Head = 'head',
  RShoulder = 'rShoulder',
  RElbow = 'rElbow',
  RWrist = 'rWrist',
  LShoulder = 'lShoulder',
  LElbow = 'lElbow',
  LWrist = 'lWrist',
  RThigh = 'rThigh',
  RSkin = 'rSkin',
  RAnkle = 'rAnkle',
  LThigh = 'lThigh',
  LSkin = 'lSkin',
  LAnkle = 'lAnkle',
  LShoulderGirdle = 'lShoulderGirdle',
  RShoulderGirdle = 'rShoulderGirdle',
  LHipGirdle = 'lHipGirdle',
  RHipGirdle = 'rHipGirdle',
  LToe = 'lToe',
  RToe = 'rToe',
}

export const PART_NAMES: PartName[] = Object.values(PartName);

export const partNameToPoseKey: { [key in PartName]: string } = {
  [PartName.Torso]: 'torso',
  [PartName.Waist]: 'waist',
  [PartName.Collar]: 'collar',
  [PartName.Head]: 'head',
  [PartName.RShoulder]: 'rShoulder',
  [PartName.RElbow]: 'rForearm',
  [PartName.RWrist]: 'rWrist',
  [PartName.LShoulder]: 'lShoulder',
  [PartName.LElbow]: 'lForearm',
  [PartName.LWrist]: 'lWrist',
  [PartName.RThigh]: 'rThigh',
  [PartName.RSkin]: 'rCalf',
  [PartName.RAnkle]: 'rAnkle',
  [PartName.LThigh]: 'lThigh',
  [PartName.LSkin]: 'lCalf',
  [PartName.LAnkle]: 'lAnkle',
  [PartName.LShoulderGirdle]: 'lShoulderGirdle',
  [PartName.RShoulderGirdle]: 'rShoulderGirdle',
  [PartName.LHipGirdle]: 'lHipGirdle',
  [PartName.RHipGirdle]: 'rHipGirdle',
  [PartName.LToe]: 'lToe',
  [PartName.RToe]: 'rToe',
};

export const PARENT_MAP: { [key in PartName]?: PartName } = {
  [PartName.Torso]: undefined, // Root is Navel
  [PartName.Waist]: undefined, // Root is Navel
  [PartName.Collar]: PartName.Torso,
  [PartName.Head]: PartName.Collar,
  [PartName.LShoulderGirdle]: PartName.Torso,
  [PartName.RShoulderGirdle]: PartName.Torso,
  [PartName.LShoulder]: PartName.LShoulderGirdle,
  [PartName.RShoulder]: PartName.RShoulderGirdle,
  [PartName.LHipGirdle]: PartName.Waist,
  [PartName.RHipGirdle]: PartName.Waist,
  [PartName.LThigh]: PartName.LHipGirdle,
  [PartName.RThigh]: PartName.RHipGirdle,
  [PartName.RElbow]: PartName.RShoulder,
  [PartName.LElbow]: PartName.LShoulder,
  [PartName.RWrist]: PartName.RElbow,
  [PartName.LWrist]: PartName.LElbow,
  [PartName.RSkin]: PartName.RThigh,
  [PartName.LSkin]: PartName.LThigh,
  [PartName.RAnkle]: PartName.RSkin,
  [PartName.LAnkle]: PartName.LSkin,
  [PartName.LToe]: PartName.LAnkle,
  [PartName.RToe]: PartName.RAnkle,
};

export const CHILD_MAP: { [key in PartName]?: PartName[] } = (() => {
  const map: { [key in PartName]?: PartName[] } = {};
  PART_NAMES.forEach(child => {
    const parent = PARENT_MAP[child];
    if (parent) {
      if (!map[parent]) map[parent] = [];
      map[parent]!.push(child);
    }
  });
  return map;
})();

export const LIMB_SEQUENCES: { [key: string]: PartName[] } = {
  rArm: [PartName.RShoulder, PartName.RElbow, PartName.RWrist],
  lArm: [PartName.LShoulder, PartName.LElbow, PartName.LWrist],
  rLeg: [PartName.RThigh, PartName.RSkin, PartName.RAnkle],
  lLeg: [PartName.LThigh, PartName.LSkin, PartName.LAnkle],
};

export type Vector2D = { x: number; y: number; };
export type Pose = {
  root: Vector2D;
  bodyRotation: number;
  torso: number;
  waist: number;
  collar: number;
  head: number;
  lShoulder: number;
  lForearm: number;
  lWrist: number;
  rShoulder: number;
  rForearm: number;
  rWrist: number;
  lThigh: number;
  lCalf: number;
  lAnkle: number;
  rThigh: number;
  rCalf: number;
  rAnkle: number;
  lShoulderGirdle: number;
  rShoulderGirdle: number;
  lHipGirdle: number;
  rHipGirdle: number;
  lToe: number;
  rToe: number;
  vOffset?: Vector2D;
  offsets?: { [key: string]: Vector2D };
};

export type FaceControls = {
  rotationEnabled: boolean;
  locked: boolean;
  manualOffset: number;
  manualEyeShift: number;
  lookBias: number;
  eyelineMode: boolean;
  dragMode: boolean;
};

export type PartVisibility = { [key in PartName]: boolean };
export type PartSelection = { [key in PartName]: boolean };
export type AnchorName = PartName | 'root' | 'lFootTip' | 'rFootTip' | 'lHandTip' | 'rHandTip'; 

export type PinnedState = {
  [key in AnchorName]?: Vector2D;
};

// Defines the available kinetic constraint modes for joints.
// Re-introduced 'stretch' and 'curl' as per user request.
export type JointConstraint = 'fk' | 'stretch' | 'curl';

export type JointLimit = { min: number; max: number };
export type JointLimits = { [key: string]: JointLimit };

// Defines the rendering mode for the Bone component.
// Simplified: 'grayscale' removed as UI is globally monochrome, 'silhouette' now represents solid black fill.
export type RenderMode = 'default' | 'wireframe' | 'silhouette' | 'backlight'; // Added 'backlight'

export type ViewMode = 'zoomed' | 'default' | 'lotte' | 'wide' | 'mobile'; // Added 'mobile'

export type KinematicMode = 'fk' | 'fabrik' | 'reactive' | 'fullBody';
export type FacingMode = 'left' | 'front' | 'right';

export type PhysicsMode = 'fluid' | 'rusted';
export type HandMode = 'neutral' | 'passive' | 'heavy';

export type TimelineKeyframe = {
  id: string;
  name: string;
  pose: Pose;
  projectTime: number; // ms on the project timeline
  active: boolean;
};

export type TimelineState = {
  keyframes: TimelineKeyframe[];
  isPlaying: boolean;
  selectedKeyframeId: string | null;
  loop: boolean;
  playbackTime: number;
};

export type SavedPose = {
  id: string;
  name: string;
  data: string;
  timestamp: number;
};

export type WalkCycleState = {
  active: boolean;
  poses: {
    contact: Pose | null;
    down: Pose | null;
    pass: Pose | null;
    up: Pose | null;
  };
  framesPerPhase: number;
  stridePx: number;
  speedFps: number;
  mirror: boolean;
  phase: number; // 0 to 1
};
