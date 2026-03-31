import { AnchorName, FaceControls, Pose, Vector2D } from '../types';

const clampRange = (value: number, min: number, max: number): number => Math.max(min, Math.min(max, value));

const computeFaceDynamics = (skullAngle: number, turnT: number) => {
  const offset = clampRange(turnT * 0.6, -1, 1);
  const eyeShift = clampRange(Math.sin(skullAngle + Math.PI / 2) * 0.6, -1, 1);
  return { dynamicOffset: offset, dynamicEyeShift: eyeShift };
};

export const getFaceShift = (
  faceControls: FaceControls,
  skullAngle: number,
  turnT: number
): { offset: number; eyeShift: number; eyelineMode: boolean } => {
  const { dynamicOffset, dynamicEyeShift } = computeFaceDynamics(skullAngle, turnT);
  let offset = faceControls.manualOffset;
  let eyeShift = faceControls.manualEyeShift;
  const allowDynamic = faceControls.rotationEnabled && !faceControls.locked;

  if (allowDynamic) {
    offset += dynamicOffset;
    if (faceControls.eyelineMode) {
      eyeShift += dynamicEyeShift;
    }
    offset += faceControls.lookBias;
  }

  offset = clampRange(offset, -1, 1);
  eyeShift = faceControls.eyelineMode ? clampRange(eyeShift, -1, 1) : 0;

  return { offset, eyeShift, eyelineMode: faceControls.eyelineMode };
};

export const applyGroundContact = (
  pose: Pose,
  joints: Record<string, Vector2D>,
  floorHeight: number
): Pose => {
  const newPose: Pose = { ...pose };
  const feet: AnchorName[] = ['lFootTip', 'rFootTip'];
  let maxPenetration = 0;

  feet.forEach(foot => {
    const pos = joints[foot as string];
    if (pos && pos.y > floorHeight) {
      maxPenetration = Math.max(maxPenetration, pos.y - floorHeight);
    }
  });

  if (maxPenetration > 0.5) {
    newPose.root = { ...newPose.root, y: newPose.root.y - maxPenetration * 0.85 };
  }

  return newPose;
};

export const applySpringFeel = (
  pose: Pose,
  joints: Record<string, Vector2D>,
  activePins: AnchorName[] = []
): Pose => {
  if (activePins.length > 0) return pose;

  const entries = Object.values(joints);
  if (!entries.length) return pose;

  const totalY = entries.reduce((sum, p) => sum + p.y, 0);
  const centerY = totalY / entries.length;
  const targetY = pose.root.y;
  const drift = centerY - targetY;

  if (Math.abs(drift) < 1) return pose;

  const newPose: Pose = { ...pose };
  newPose.root = { ...newPose.root, y: newPose.root.y - drift * 0.04 };
  return newPose;
};

export const applyCoMStabilizer = (
  pose: Pose,
  joints: Record<string, Vector2D>,
  activePins: AnchorName[],
  margin: number = 18
): Pose => {
  const supportingX: number[] = [];
  activePins.forEach(pin => {
    const pos = joints[pin as string];
    if (pos) supportingX.push(pos.x);
  });

  if (!supportingX.length) return pose;

  const left = Math.min(...supportingX) - margin;
  const right = Math.max(...supportingX) + margin;
  const comX = pose.root.x;
  let correction = 0;

  if (comX < left) {
    correction = (comX - left) * 0.18;
  } else if (comX > right) {
    correction = (comX - right) * 0.18;
  }

  if (!correction) return pose;

  const newPose: Pose = { ...pose };
  newPose.bodyRotation -= correction * 0.004;
  newPose.root = { ...newPose.root, x: newPose.root.x - correction * 0.12 };
  return newPose;
};
