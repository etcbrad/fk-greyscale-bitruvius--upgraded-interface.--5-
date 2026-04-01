
import { applyCoMStabilizer, applyGroundContact, applySpringFeel, getFaceShift } from './puppet-core';
import { ANATOMY, BASE_ROTATIONS, RIGGING, JOINT_LIMITS, FLOOR_HEIGHT } from '../constants';
import { PartName, Pose, Vector2D, AnchorName, JointConstraint, CHILD_MAP, LIMB_SEQUENCES, partNameToPoseKey, FaceControls, FacingMode, HandMode, KinematicMode } from '../types';

export const lerp = (start: number, end: number, t: number): number => start * (1 - t) + end * t;
const mixVec = (a: Vector2D, b: Vector2D, t: number): Vector2D => ({
  x: lerp(a.x, b.x, t),
  y: lerp(a.y, b.y, t),
});

const DEFAULT_FACE_CONTROLS: FaceControls = {
  rotationEnabled: true,
  locked: false,
  manualOffset: 0,
  manualEyeShift: 0,
  lookBias: 0,
  eyelineMode: true,
  dragMode: false,
};

/**
 * Calculates shortest angular difference between two angles in degrees.
 */
export const getShortestAngleDiffDeg = (currentDeg: number, startDeg: number): number => {
  let diff = currentDeg - startDeg;
  diff = ((diff % 360) + 360) % 360; 
  if (diff > 180) diff -= 360;
  return diff;
};

export const lerpAngleShortestPath = (a: number, b: number, t: number): number => {
  const normalize = (angle: number) => ((angle % 360) + 360) % 360;
  let start = normalize(a);
  let end = normalize(b);
  let delta = end - start;
  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;
  return a + delta * t;
};

const rad = (deg: number): number => deg * Math.PI / 180;
const deg = (rad: number): number => rad * 180 / Math.PI;
export const dist = (v1: Vector2D, v2: Vector2D): number => Math.sqrt(Math.pow(v2.x - v1.x, 2) + Math.pow(v2.y - v1.y, 2));
const safeRatio = (length: number, divisor: number): number => divisor < 1e-6 ? 0 : length / divisor;

const rotateVec = (x: number, y: number, angleDeg: number): Vector2D => {
  const r = rad(angleDeg);
  const c = Math.cos(r);
  const s = Math.sin(r);
  return { x: x * c - y * s, y: x * s + y * c };
};
const addVec = (v1: Vector2D, v2: Vector2D): Vector2D => ({ x: v1.x + v2.x, y: v1.y + v2.y });

export const getTotalRotation = (key: string, pose: Pose): number => 
  (BASE_ROTATIONS[key as keyof typeof BASE_ROTATIONS] || 0) + ((pose as any)[key] || 0);

const calculateBoneGlobalPositions = (
  parentGlobalPos: Vector2D,
  parentGlobalAngle: number,
  boneTotalLocalRotation: number,
  boneLength: number,
  boneOffset: Vector2D = { x: 0, y: 0 },
  isUpwardDrawing: boolean = false
): { globalStartPoint: Vector2D; globalEndPoint: Vector2D; childInheritedGlobalAngle: number } => {
  const rotatedOffset = rotateVec(boneOffset.x, boneOffset.y, parentGlobalAngle);
  const globalStartPoint = addVec(parentGlobalPos, rotatedOffset);
  const boneGlobalAngle = parentGlobalAngle + boneTotalLocalRotation;
  const y_dir = isUpwardDrawing ? -1 : 1;
  const boneVector = rotateVec(0, boneLength * y_dir, boneGlobalAngle);
  const globalEndPoint = addVec(globalStartPoint, boneVector);
  return { globalStartPoint, globalEndPoint, childInheritedGlobalAngle: boneGlobalAngle };
};

/**
 * Apply Curl/Stretch kinetic behaviors to child joints when a parent joint rotates.
 */
export const applyKineticBehaviors = (
  pose: Pose,
  changedPart: PartName,
  angleDelta: number,
  jointModes: Record<PartName, JointConstraint>,
  resonance: number = 1.0
): Pose => {
  const newPose = { ...pose };
  const children = CHILD_MAP[changedPart];
  if (!children) return newPose;

  children.forEach(child => {
    const mode = jointModes[child];
    if (mode === 'fk') return;

    const poseKey = child === PartName.RElbow ? 'rForearm' : 
                    child === PartName.LElbow ? 'lForearm' : 
                    child === PartName.RSkin ? 'rCalf' : 
                    child === PartName.LSkin ? 'lCalf' : 
                    (child as string);

    if (mode === 'stretch') {
      // Counter-rotate to maintain world orientation
      (newPose as any)[poseKey] = ((newPose as any)[poseKey] || 0) - (angleDelta * resonance);
    } else if (mode === 'curl') {
      // Exaggerate fold (rotate same direction as parent)
      (newPose as any)[poseKey] = ((newPose as any)[poseKey] || 0) + (angleDelta * resonance);
    }
  });

  return newPose;
};

const _calculateGlobalJointPositions = (
    baseRoot: Vector2D,
    baseBodyRotation: number,
    pose: Pose
): Record<string, Vector2D> => {
    const offsets = pose.offsets || {};

    // Torso and Waist are now siblings parented to the Navel (baseRoot)
    const torsoCalc = calculateBoneGlobalPositions(baseRoot, baseBodyRotation, getTotalRotation(PartName.Torso, pose), ANATOMY.TORSO, offsets[PartName.Torso], true);
    const waistCalc = calculateBoneGlobalPositions(baseRoot, baseBodyRotation, getTotalRotation(PartName.Waist, pose), ANATOMY.WAIST, offsets[PartName.Waist], false);
    
    // Neck (Collar) connects to Torso at the V joint (torso end)
    // Keep the neck welded to the top of the spine; it pivots at the V joint and does not slide across the shoulders.
    const collarCalc = calculateBoneGlobalPositions(torsoCalc.globalEndPoint, torsoCalc.childInheritedGlobalAngle, getTotalRotation(PartName.Collar, pose), ANATOMY.COLLAR, { x: 0, y: 0 }, true);
    const collarAngle = collarCalc.childInheritedGlobalAngle;
    const collarEnd = collarCalc.globalEndPoint;

    // Head connects to Neck at the Chin joint (collar end)
    const headPivot = addVec(collarEnd, rotateVec(0, -ANATOMY.HEAD_NECK_GAP_OFFSET, collarAngle));
    const headGlobalAngle = collarAngle + getTotalRotation(PartName.Head, pose);
    const headTip = addVec(headPivot, rotateVec(0, -ANATOMY.HEAD, headGlobalAngle));

    // Side Bones (Shoulder Girdles) connect at Navel
    const getGirdleAngle = (isRight: boolean) => {
        const defaultAngle = Math.atan2(isRight ? ANATOMY.SHOULDER_WIDTH / 2 : -ANATOMY.SHOULDER_WIDTH / 2, ANATOMY.TORSO) * 180 / Math.PI;
        return defaultAngle;
    };
    const girdleLen = Math.sqrt(Math.pow(ANATOMY.SHOULDER_WIDTH / 2, 2) + Math.pow(ANATOMY.TORSO, 2));

    const getArmJoints = (isRight: boolean) => {
        const partGirdle = isRight ? PartName.RShoulderGirdle : PartName.LShoulderGirdle;
        const girdleAngle = getGirdleAngle(isRight);
        const girdleCalc = calculateBoneGlobalPositions(baseRoot, torsoCalc.childInheritedGlobalAngle, girdleAngle + getTotalRotation(partGirdle, pose), girdleLen, offsets[partGirdle], true);
        
        const torsoShoulderAttach = girdleCalc.globalEndPoint;
        const waistShoulderAttach = addVec(baseRoot, rotateVec(isRight ? RIGGING.R_SHOULDER_X_OFFSET_FROM_COLLAR_CENTER : RIGGING.L_SHOULDER_X_OFFSET_FROM_COLLAR_CENTER, RIGGING.SHOULDER_Y_OFFSET_FROM_COLLAR_END, waistCalc.childInheritedGlobalAngle));
        const collarBalanceFactor = Math.min(1, Math.abs(getTotalRotation(PartName.Collar, pose)) / 55);
        const torsoBalanceFactor = Math.min(1, Math.abs(getTotalRotation(PartName.Torso, pose)) / 70);
        const rubberBlend = 0.18 + collarBalanceFactor * 0.3 + torsoBalanceFactor * 0.12;
        const shoulderAttach = mixVec(torsoShoulderAttach, waistShoulderAttach, rubberBlend);
        const upperArmCalc = calculateBoneGlobalPositions(shoulderAttach, girdleCalc.childInheritedGlobalAngle, getTotalRotation(isRight ? PartName.RShoulder : PartName.LShoulder, pose), ANATOMY.UPPER_ARM, offsets[isRight ? PartName.RShoulder : PartName.LShoulder], false);
        const forearmCalc = calculateBoneGlobalPositions(upperArmCalc.globalEndPoint, upperArmCalc.childInheritedGlobalAngle, getTotalRotation(isRight ? 'rForearm' : 'lForearm', pose), ANATOMY.LOWER_ARM, offsets[isRight ? PartName.RElbow : PartName.LElbow], false);
        const handAngle = forearmCalc.childInheritedGlobalAngle + getTotalRotation(isRight ? PartName.RWrist : PartName.LWrist, pose);
        const handTip = addVec(forearmCalc.globalEndPoint, rotateVec(0, ANATOMY.HAND, handAngle));
        
        return { girdle: girdleCalc.globalStartPoint, shoulder: shoulderAttach, elbow: upperArmCalc.globalEndPoint, wrist: forearmCalc.globalEndPoint, hand: handTip, girdleAngle: girdleCalc.childInheritedGlobalAngle };
    };

    const getHipGirdleAngle = (isRight: boolean) => {
        const defaultAngle = Math.atan2(isRight ? ANATOMY.HIP_WIDTH / 2 : -ANATOMY.HIP_WIDTH / 2, ANATOMY.WAIST) * 180 / Math.PI;
        return defaultAngle;
    };
    const hipGirdleLen = Math.sqrt(Math.pow(ANATOMY.HIP_WIDTH / 2, 2) + Math.pow(ANATOMY.WAIST, 2));

    const getLegJoints = (isRight: boolean) => {
        const partGirdle = isRight ? PartName.RHipGirdle : PartName.LHipGirdle;
        const girdleAngle = getHipGirdleAngle(isRight);
        const girdleCalc = calculateBoneGlobalPositions(baseRoot, waistCalc.childInheritedGlobalAngle, girdleAngle + getTotalRotation(partGirdle, pose), hipGirdleLen, offsets[partGirdle], false);
        
        const waistHipAttach = girdleCalc.globalEndPoint;
        const lowerBodyBalanceFactor = Math.min(1, Math.abs(getTotalRotation(PartName.Waist, pose)) / 45);
        const rootHipAttach = addVec(baseRoot, rotateVec(isRight ? ANATOMY.HIP_WIDTH * 0.22 : -ANATOMY.HIP_WIDTH * 0.22, ANATOMY.WAIST * 0.92, waistCalc.childInheritedGlobalAngle));
        const hipAttach = mixVec(waistHipAttach, rootHipAttach, 0.15 + lowerBodyBalanceFactor * 0.25);
        const thighCalc = calculateBoneGlobalPositions(hipAttach, girdleCalc.childInheritedGlobalAngle, getTotalRotation(isRight ? PartName.RThigh : PartName.LThigh, pose), ANATOMY.LEG_UPPER, offsets[isRight ? PartName.RThigh : PartName.LThigh], false);
        const calfCalc = calculateBoneGlobalPositions(thighCalc.globalEndPoint, thighCalc.childInheritedGlobalAngle, getTotalRotation(isRight ? 'rCalf' : 'lCalf', pose), ANATOMY.LEG_LOWER, offsets[isRight ? PartName.RSkin : PartName.LSkin], false);
        const ankleAngle = calfCalc.childInheritedGlobalAngle + getTotalRotation(isRight ? PartName.RAnkle : PartName.LAnkle, pose);
        const footEnd = addVec(calfCalc.globalEndPoint, rotateVec(0, ANATOMY.FOOT, ankleAngle));
        const toeAngle = ankleAngle + getTotalRotation(isRight ? PartName.RToe : PartName.LToe, pose);
        const toeTip = addVec(footEnd, rotateVec(0, ANATOMY.TOE, toeAngle));
        
        return { hipGirdle: girdleCalc.globalStartPoint, hip: hipAttach, knee: thighCalc.globalEndPoint, ankle: calfCalc.globalEndPoint, footTip: toeTip, footEnd };
    };

    const rArm = getArmJoints(true);
    const lArm = getArmJoints(false);
    const rLeg = getLegJoints(true);
    const lLeg = getLegJoints(false);

    return {
        root: baseRoot,
        [PartName.Waist]: baseRoot,
        [PartName.Torso]: baseRoot,
        [PartName.Collar]: torsoCalc.globalEndPoint,
        [PartName.Head]: headPivot,
        [PartName.RShoulderGirdle]: rArm.girdle,
        [PartName.RShoulder]: rArm.shoulder,
        [PartName.RElbow]: rArm.elbow,
        [PartName.RWrist]: rArm.wrist,
        [PartName.LShoulderGirdle]: lArm.girdle,
        [PartName.LShoulder]: lArm.shoulder,
        [PartName.LElbow]: lArm.elbow,
        [PartName.LWrist]: lArm.wrist,
        [PartName.RHipGirdle]: rLeg.hipGirdle,
        [PartName.RThigh]: rLeg.hip,
        [PartName.RSkin]: rLeg.knee,
        [PartName.RAnkle]: rLeg.ankle,
        [PartName.RToe]: rLeg.footEnd,
        [PartName.LHipGirdle]: lLeg.hipGirdle,
        [PartName.LThigh]: lLeg.hip,
        [PartName.LSkin]: lLeg.knee,
        [PartName.LAnkle]: lLeg.ankle,
        [PartName.LToe]: lLeg.footEnd,
        headTip,
        rFootTip: rLeg.footTip,
        lFootTip: lLeg.footTip,
        rHandTip: rArm.hand,
        lHandTip: lArm.hand,
    };
};

/**
 * Calculates global positions of all joints, adjusted for the active pins.
 * The primary pin (first in array) is used for stabilization.
 * Other pins will exhibit "elasticity" (tension) if the model moves away from them.
 */
export const getJointPositions = (pose: Pose, activePins: AnchorName[]): Record<string, Vector2D> => {
    const inputRoot = pose.root;
    const inputBodyRotation = getTotalRotation('bodyRotation', pose);
    const primaryPin = activePins[0] || 'root';

    // If pinning root, just return standard calc.
    if (primaryPin === 'root' || primaryPin === PartName.Waist) {
        return _calculateGlobalJointPositions(inputRoot, inputBodyRotation, pose);
    }

    // To keep a pin fixed while rotating, we find the offset the rotation caused.
    const jointsNoRot = _calculateGlobalJointPositions(inputRoot, 0, pose);
    const pinNoRot = jointsNoRot[primaryPin as string];
    if (!pinNoRot) return _calculateGlobalJointPositions(inputRoot, inputBodyRotation, pose);

    const jointsWithRot = _calculateGlobalJointPositions(inputRoot, inputBodyRotation, pose);
    const pinWithRot = jointsWithRot[primaryPin as string];
    if (!pinWithRot) return jointsWithRot;

    // Calculate how much the pin moved due to rotation
    const offset = {
        x: pinNoRot.x - pinWithRot.x,
        y: pinNoRot.y - pinWithRot.y,
    };

    // Shift the entire model by that offset to keep the pin at its "no-rotation" position
    const stabilizedRoot = {
        x: inputRoot.x + offset.x,
        y: inputRoot.y + offset.y,
    };

    return _calculateGlobalJointPositions(stabilizedRoot, inputBodyRotation, pose);
};

/**
 * Calculates the tension factor (0 to 1+) based on distance from pin to anatomical joint.
 */
export const calculateTensionFactor = (anatomicalPos: Vector2D, pinnedPos: Vector2D, threshold: number = 50): number => {
    const dx = anatomicalPos.x - pinnedPos.x;
    const dy = anatomicalPos.y - pinnedPos.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    return Math.min(2.0, distance / threshold);
};

/**
 * Interpolates between two poses.
 */
export const interpolatePoses = (start: Pose, end: Pose, t: number): Pose => {
  const result: any = {
    root: {
      x: lerp(start.root.x, end.root.x, t),
      y: lerp(start.root.y, end.root.y, t),
    },
    bodyRotation: lerpAngleShortestPath(start.bodyRotation, end.bodyRotation, t),
    vOffset: {
      x: lerp(start.vOffset?.x || 0, end.vOffset?.x || 0, t),
      y: lerp(start.vOffset?.y || 0, end.vOffset?.y || 0, t),
    },
    offsets: {},
  };

  const keys = Object.keys(BASE_ROTATIONS) as (keyof typeof BASE_ROTATIONS)[];
  keys.forEach(key => {
    if (key === 'bodyRotation') return;
    result[key] = lerpAngleShortestPath((start as any)[key] || 0, (end as any)[key] || 0, t);
  });

  return result as Pose;
};

const normalizeAngleDeg = (angle: number): number => ((angle + 180) % 360 + 360) % 360 - 180;
const getClosestEquivalentAngleDeg = (target: number, reference: number): number => {
  const normalizedTarget = normalizeAngleDeg(target);
  const normalizedReference = normalizeAngleDeg(reference);
  return normalizedReference + getShortestAngleDiffDeg(normalizedTarget, normalizedReference);
};

const getFabrikSolveChain = (
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName
): PartName[] | null => {
  const chain = LIMB_SEQUENCES[limbName];
  const effectorIndex = chain?.indexOf(effectorPart) ?? -1;
  if (!chain || effectorIndex < 1) return null;
  return chain.slice(0, effectorIndex + 1);
};

const getFabrikParentAngle = (
  pose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg'
): number => {
  const bodyAngle = getTotalRotation('bodyRotation', pose);
  if (limbName === 'rArm') {
    return bodyAngle + getTotalRotation(PartName.Torso, pose) + getTotalRotation(PartName.RShoulderGirdle, pose);
  }
  if (limbName === 'lArm') {
    return bodyAngle + getTotalRotation(PartName.Torso, pose) + getTotalRotation(PartName.LShoulderGirdle, pose);
  }
  if (limbName === 'rLeg') {
    return bodyAngle + getTotalRotation(PartName.Waist, pose) + getTotalRotation(PartName.RHipGirdle, pose);
  }
  return bodyAngle + getTotalRotation(PartName.Waist, pose) + getTotalRotation(PartName.LHipGirdle, pose);
};

const preserveElbowDownstreamOrientation = (
  nextPose: Pose,
  originalPose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  activePins: AnchorName[]
) => {
  if (limbName !== 'rArm' && limbName !== 'lArm') return;

  const isRight = limbName === 'rArm';
  const forearmKey = isRight ? 'rForearm' : 'lForearm';
  const wristPart = isRight ? PartName.RWrist : PartName.LWrist;
  const elbowPart = isRight ? PartName.RElbow : PartName.LElbow;

  const originalJoints = getJointPositions(originalPose, activePins);
  const originalElbow = originalJoints[elbowPart];
  const originalWrist = originalJoints[wristPart];
  const originalHandTip = originalJoints[isRight ? 'rHandTip' : 'lHandTip'];
  if (!originalElbow || !originalWrist || !originalHandTip) return;

  const originalForearmGlobal = deg(Math.atan2(originalWrist.y - originalElbow.y, originalWrist.x - originalElbow.x)) - 90;
  const originalHandGlobal = deg(Math.atan2(originalHandTip.y - originalWrist.y, originalHandTip.x - originalWrist.x)) - 90;

  const elbowParentAngle =
    getFabrikParentAngle(nextPose, limbName) +
    (BASE_ROTATIONS[partNameToPoseKey[limbName === 'rArm' ? PartName.RShoulder : PartName.LShoulder] as keyof typeof BASE_ROTATIONS] || 0) +
    ((nextPose as any)[limbName === 'rArm' ? 'rShoulder' : 'lShoulder'] || 0);

  let forearmLocal = normalizeAngleDeg(
    originalForearmGlobal - elbowParentAngle - (BASE_ROTATIONS[forearmKey as keyof typeof BASE_ROTATIONS] || 0)
  );
  const forearmLimits = JOINT_LIMITS[forearmKey];
  if (forearmLimits) {
    forearmLocal = Math.max(forearmLimits.min, Math.min(forearmLimits.max, forearmLocal));
  }
  (nextPose as any)[forearmKey] = forearmLocal;

  const wristParentAngle = elbowParentAngle + (BASE_ROTATIONS[forearmKey as keyof typeof BASE_ROTATIONS] || 0) + forearmLocal;
  const wristPoseKey = partNameToPoseKey[wristPart];
  let wristLocal = normalizeAngleDeg(
    originalHandGlobal - wristParentAngle - (BASE_ROTATIONS[wristPoseKey as keyof typeof BASE_ROTATIONS] || 0)
  );
  const wristLimits = JOINT_LIMITS[wristPoseKey];
  if (wristLimits) {
    wristLocal = Math.max(wristLimits.min, Math.min(wristLimits.max, wristLocal));
  }
  (nextPose as any)[wristPoseKey] = wristLocal;
};

const projectPointToCircle = (center: Vector2D, radius: number, target: Vector2D): Vector2D => {
  const dx = target.x - center.x;
  const dy = target.y - center.y;
  const distance = Math.hypot(dx, dy);
  if (distance < 1e-6) {
    return { x: center.x, y: center.y + radius };
  }
  const scale = radius / distance;
  return {
    x: center.x + dx * scale,
    y: center.y + dy * scale,
  };
};

const solveArmElbowGuide = (
  pose: Pose,
  limbName: 'rArm' | 'lArm',
  target: Vector2D,
  activePins: AnchorName[]
): Pose => {
  const elbowGuideSensitivity = 0.22;
  const wristPreservationBlend = 0.18;
  const isRight = limbName === 'rArm';
  const shoulderPart = isRight ? PartName.RShoulder : PartName.LShoulder;
  const elbowPart = isRight ? PartName.RElbow : PartName.LElbow;
  const wristPart = isRight ? PartName.RWrist : PartName.LWrist;
  const handTipKey = isRight ? 'rHandTip' : 'lHandTip';
  const forearmKey = isRight ? 'rForearm' : 'lForearm';
  const wristPoseKey = partNameToPoseKey[wristPart];

  const originalJoints = getJointPositions(pose, activePins);
  const shoulder = originalJoints[shoulderPart];
  const originalElbow = originalJoints[elbowPart];
  const originalWrist = originalJoints[wristPart];
  const originalHandTip = originalJoints[handTipKey];
  if (!shoulder || !originalElbow || !originalWrist || !originalHandTip) return pose;

  const nextPose = { ...pose };
  const refinedTarget = {
    x: lerp(originalElbow.x, target.x, elbowGuideSensitivity),
    y: lerp(originalElbow.y, target.y, elbowGuideSensitivity),
  };
  const newElbow = projectPointToCircle(shoulder, ANATOMY.UPPER_ARM, refinedTarget);

  const shoulderParentAngle = getFabrikParentAngle(nextPose, limbName);
  const shoulderPoseKey = partNameToPoseKey[shoulderPart];
  const shoulderBaseRotation = BASE_ROTATIONS[shoulderPoseKey as keyof typeof BASE_ROTATIONS] || 0;
  const upperArmGlobal = deg(Math.atan2(newElbow.y - shoulder.y, newElbow.x - shoulder.x)) - 90;
  const currentShoulderLocal = (pose as any)[shoulderPoseKey] || 0;
  let shoulderLocal = getClosestEquivalentAngleDeg(
    upperArmGlobal - shoulderParentAngle - shoulderBaseRotation,
    currentShoulderLocal
  );
  const shoulderLimits = JOINT_LIMITS[shoulderPoseKey];
  if (shoulderLimits) {
    shoulderLocal = Math.max(shoulderLimits.min, Math.min(shoulderLimits.max, shoulderLocal));
  }
  (nextPose as any)[shoulderPoseKey] = shoulderLocal;

  const updatedJoints = getJointPositions(nextPose, activePins);
  const resolvedShoulder = updatedJoints[shoulderPart];
  if (!resolvedShoulder) return pose;
  const resolvedElbow = projectPointToCircle(resolvedShoulder, ANATOMY.UPPER_ARM, refinedTarget);
  const newWrist = projectPointToCircle(resolvedElbow, ANATOMY.LOWER_ARM, originalWrist);

  const elbowParentAngle =
    getFabrikParentAngle(nextPose, limbName) +
    shoulderBaseRotation +
    shoulderLocal;
  const forearmBaseRotation = BASE_ROTATIONS[forearmKey as keyof typeof BASE_ROTATIONS] || 0;
  const forearmGlobal = deg(Math.atan2(newWrist.y - resolvedElbow.y, newWrist.x - resolvedElbow.x)) - 90;
  const currentForearmLocal = (pose as any)[forearmKey] || 0;
  let forearmLocal = getClosestEquivalentAngleDeg(
    forearmGlobal - elbowParentAngle - forearmBaseRotation,
    currentForearmLocal
  );
  const forearmLimits = JOINT_LIMITS[forearmKey];
  if (forearmLimits) {
    forearmLocal = Math.max(forearmLimits.min, Math.min(forearmLimits.max, forearmLocal));
  }
  (nextPose as any)[forearmKey] = forearmLocal;

  const originalHandGlobal = deg(Math.atan2(originalHandTip.y - originalWrist.y, originalHandTip.x - originalWrist.x)) - 90;
  const wristParentAngle = elbowParentAngle + forearmBaseRotation + forearmLocal;
  const wristBaseRotation = BASE_ROTATIONS[wristPoseKey as keyof typeof BASE_ROTATIONS] || 0;
  const currentWristLocal = (pose as any)[wristPoseKey] || 0;
  const desiredWristLocal = getClosestEquivalentAngleDeg(
    originalHandGlobal - wristParentAngle - wristBaseRotation,
    currentWristLocal
  );
  let wristLocal = lerp(currentWristLocal, desiredWristLocal, wristPreservationBlend);
  const wristLimits = JOINT_LIMITS[wristPoseKey];
  if (wristLimits) {
    wristLocal = Math.max(wristLimits.min, Math.min(wristLimits.max, wristLocal));
  }
  (nextPose as any)[wristPoseKey] = wristLocal;

  return nextPose;
};

const getLimbFacingBendDirection = (
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  facingMode: FacingMode
): number => {
  if (facingMode === 'left') return -1;
  if (facingMode === 'right') return 1;
  return limbName === 'rArm' || limbName === 'rLeg' ? 1 : -1;
};

const applyFacingBendPreference = (
  points: Vector2D[],
  lengths: number[],
  target: Vector2D,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  facingMode: FacingMode
) => {
  if (points.length !== 3 || lengths.length !== 2) return;

  const root = points[0];
  const effector = target;
  const toEffector = { x: effector.x - root.x, y: effector.y - root.y };
  const distanceToEffector = Math.hypot(toEffector.x, toEffector.y);
  if (distanceToEffector < 1e-6) return;

  const clampedDistance = Math.min(distanceToEffector, lengths[0] + lengths[1] - 1e-6);
  const direction = { x: toEffector.x / distanceToEffector, y: toEffector.y / distanceToEffector };
  const along = (lengths[0] * lengths[0] - lengths[1] * lengths[1] + clampedDistance * clampedDistance) / (2 * clampedDistance);
  const perpendicularMagnitude = Math.sqrt(Math.max(lengths[0] * lengths[0] - along * along, 0));
  const normal = { x: -direction.y, y: direction.x };
  const bendDirection = getLimbFacingBendDirection(limbName, facingMode);

  points[1] = {
    x: root.x + direction.x * along + normal.x * perpendicularMagnitude * bendDirection,
    y: root.y + direction.y * along + normal.y * perpendicularMagnitude * bendDirection,
  };
  points[2] = { ...target };
};

export const solveFABRIK = (
  pose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName,
  target: Vector2D,
  activePins: AnchorName[],
  facingMode: FacingMode = 'front',
  iterations: number = 16,
  tolerance: number = 0.5
): Pose => {
  if ((effectorPart === PartName.RElbow || effectorPart === PartName.LElbow) && (limbName === 'rArm' || limbName === 'lArm')) {
    return solveArmElbowGuide(pose, limbName, target, activePins);
  }

  const chain = getFabrikSolveChain(limbName, effectorPart);
  if (!chain) return pose;

  const nextPose = { ...pose };
  const joints = getJointPositions(nextPose, activePins);
  const points = chain.map((joint) => ({ ...joints[joint as string] }));
  if (points.some((point) => !Number.isFinite(point.x) || !Number.isFinite(point.y))) {
    return pose;
  }

  const lengths: number[] = [];
  for (let i = 0; i < points.length - 1; i++) {
    lengths.push(dist(points[i], points[i + 1]));
  }

  const origin = { ...points[0] };
  const totalLength = lengths.reduce((sum, length) => sum + length, 0);
  const targetDist = dist(origin, target);
  if (totalLength < 1e-6) return pose;

  if (targetDist >= totalLength - tolerance) {
    for (let i = 0; i < points.length - 1; i++) {
      const distanceToTarget = dist(points[i], target);
      const lambda = safeRatio(lengths[i], distanceToTarget);
      points[i + 1] = {
        x: (1 - lambda) * points[i].x + lambda * target.x,
        y: (1 - lambda) * points[i].y + lambda * target.y,
      };
    }
  } else {
    for (let iteration = 0; iteration < iterations; iteration++) {
      if (dist(points[points.length - 1], target) <= tolerance) break;

      points[points.length - 1] = { ...target };
      for (let i = points.length - 2; i >= 0; i--) {
        const distance = dist(points[i + 1], points[i]);
        const lambda = safeRatio(lengths[i], distance);
        points[i] = {
          x: (1 - lambda) * points[i + 1].x + lambda * points[i].x,
          y: (1 - lambda) * points[i + 1].y + lambda * points[i].y,
        };
      }

      points[0] = { ...origin };
      for (let i = 0; i < points.length - 1; i++) {
        const distance = dist(points[i], points[i + 1]);
        const lambda = safeRatio(lengths[i], distance);
        points[i + 1] = {
          x: (1 - lambda) * points[i].x + lambda * points[i + 1].x,
          y: (1 - lambda) * points[i].y + lambda * points[i + 1].y,
        };
      }
    }
  }

  applyFacingBendPreference(points, lengths, target, limbName, facingMode);

  let parentAngle = getFabrikParentAngle(nextPose, limbName);
  for (let i = 0; i < chain.length - 1; i++) {
    const part = chain[i];
    const poseKey = partNameToPoseKey[part];
    const baseRotation = BASE_ROTATIONS[poseKey as keyof typeof BASE_ROTATIONS] || 0;
    const directionAngle = deg(Math.atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x));
    const segmentRotation = directionAngle - 90;

    let localAngle = normalizeAngleDeg(segmentRotation - parentAngle - baseRotation);
    const limits = JOINT_LIMITS[poseKey as keyof typeof JOINT_LIMITS];
    if (limits) {
      localAngle = Math.max(limits.min, Math.min(limits.max, localAngle));
    }

    (nextPose as any)[poseKey] = localAngle;
    parentAngle += baseRotation + localAngle;
  }

  if (effectorPart === PartName.RElbow || effectorPart === PartName.LElbow) {
    preserveElbowDownstreamOrientation(nextPose, pose, limbName, activePins);
  }

  return nextPose;
};

const clampValue = (value: number, min: number, max: number): number => Math.max(min, Math.min(max, value));

const applyDeltaToPoseKey = (pose: Pose, key: keyof Pose, delta: number) => {
  if (typeof pose[key] !== 'number' || !Number.isFinite(delta)) return;
  const current = pose[key] as number;
  const limits = JOINT_LIMITS[key as string];
  const next = current + delta;
  (pose as any)[key] = limits ? clampValue(next, limits.min, limits.max) : next;
};

const getAnchorSolveSpec = (
  pose: Pose,
  anchor: AnchorName,
  target: Vector2D,
  activePins: AnchorName[]
): { limb: 'rArm' | 'lArm' | 'rLeg' | 'lLeg'; effector: PartName; target: Vector2D } | null => {
  const joints = getJointPositions(pose, activePins);

  if (anchor === PartName.RWrist || anchor === 'rHandTip') {
    const wrist = joints[PartName.RWrist];
    const tip = joints.rHandTip;
    const wristToTip = wrist && tip ? { x: tip.x - wrist.x, y: tip.y - wrist.y } : { x: 0, y: 0 };
    return {
      limb: 'rArm',
      effector: PartName.RWrist,
      target: anchor === 'rHandTip' ? { x: target.x - wristToTip.x, y: target.y - wristToTip.y } : target,
    };
  }
  if (anchor === PartName.LWrist || anchor === 'lHandTip') {
    const wrist = joints[PartName.LWrist];
    const tip = joints.lHandTip;
    const wristToTip = wrist && tip ? { x: tip.x - wrist.x, y: tip.y - wrist.y } : { x: 0, y: 0 };
    return {
      limb: 'lArm',
      effector: PartName.LWrist,
      target: anchor === 'lHandTip' ? { x: target.x - wristToTip.x, y: target.y - wristToTip.y } : target,
    };
  }
  if (anchor === PartName.RElbow) return { limb: 'rArm', effector: PartName.RElbow, target };
  if (anchor === PartName.LElbow) return { limb: 'lArm', effector: PartName.LElbow, target };
  if (anchor === PartName.RAnkle || anchor === 'rFootTip') {
    const ankle = joints[PartName.RAnkle];
    const tip = joints.rFootTip;
    const ankleToTip = ankle && tip ? { x: tip.x - ankle.x, y: tip.y - ankle.y } : { x: 0, y: 0 };
    return {
      limb: 'rLeg',
      effector: PartName.RAnkle,
      target: anchor === 'rFootTip' ? { x: target.x - ankleToTip.x, y: target.y - ankleToTip.y } : target,
    };
  }
  if (anchor === PartName.LAnkle || anchor === 'lFootTip') {
    const ankle = joints[PartName.LAnkle];
    const tip = joints.lFootTip;
    const ankleToTip = ankle && tip ? { x: tip.x - ankle.x, y: tip.y - ankle.y } : { x: 0, y: 0 };
    return {
      limb: 'lLeg',
      effector: PartName.LAnkle,
      target: anchor === 'lFootTip' ? { x: target.x - ankleToTip.x, y: target.y - ankleToTip.y } : target,
    };
  }
  if (anchor === PartName.RSkin) return { limb: 'rLeg', effector: PartName.RSkin, target };
  if (anchor === PartName.LSkin) return { limb: 'lLeg', effector: PartName.LSkin, target };
  return null;
};

const isAnchorOnSameLimb = (anchor: AnchorName, limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg'): boolean => {
  if (limbName === 'rArm') return [PartName.RElbow, PartName.RWrist, 'rHandTip'].includes(anchor as any);
  if (limbName === 'lArm') return [PartName.LElbow, PartName.LWrist, 'lHandTip'].includes(anchor as any);
  if (limbName === 'rLeg') return [PartName.RSkin, PartName.RAnkle, 'rFootTip'].includes(anchor as any);
  return [PartName.LSkin, PartName.LAnkle, 'lFootTip'].includes(anchor as any);
};

const preservePinnedAnchors = (
  pose: Pose,
  originalPose: Pose,
  activePins: AnchorName[],
  draggedLimb: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  facingMode: FacingMode,
  passes: number
): Pose => {
  let nextPose = { ...pose };
  const originalJoints = getJointPositions(originalPose, activePins);

  for (let pass = 0; pass < passes; pass++) {
    for (const anchor of activePins) {
      if (anchor === 'root' || anchor === PartName.Waist || isAnchorOnSameLimb(anchor, draggedLimb)) continue;
      const target = originalJoints[anchor as string];
      if (!target) continue;
      const spec = getAnchorSolveSpec(nextPose, anchor, target, activePins);
      if (!spec) continue;
      nextPose = solveFABRIK(nextPose, spec.limb, spec.effector, spec.target, [], facingMode, 14, 0.5);
      if (spec.limb === 'lLeg') nextPose = solveGroundAwareAnkle(nextPose, false, [], target.y + 1);
      if (spec.limb === 'rLeg') nextPose = solveGroundAwareAnkle(nextPose, true, [], target.y + 1);
    }
  }

  return nextPose;
};

const applyWholeBodyCompensation = (
  pose: Pose,
  originalPose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName,
  activePins: AnchorName[],
  strength: number
): Pose => {
  const originalJoints = getJointPositions(originalPose, activePins);
  const updatedJoints = getJointPositions(pose, activePins);
  const originalEffector = originalJoints[effectorPart];
  const updatedEffector = updatedJoints[effectorPart];
  if (!originalEffector || !updatedEffector) return pose;

  const dx = updatedEffector.x - originalEffector.x;
  const dy = updatedEffector.y - originalEffector.y;
  const side = limbName.startsWith('r') ? 1 : -1;
  const nextPose = { ...pose };

  if (limbName.includes('Arm')) {
    applyDeltaToPoseKey(nextPose, 'torso', clampValue(side * dx * 0.022 * strength, -22, 22));
    applyDeltaToPoseKey(nextPose, 'waist', clampValue(side * dx * 0.012 * strength, -14, 14));
    applyDeltaToPoseKey(nextPose, 'collar', clampValue(side * dx * 0.028 * strength - dy * 0.018 * strength, -26, 26));
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rArm' ? 'rShoulderGirdle' : 'lShoulderGirdle',
      clampValue(side * dx * 0.035 * strength - dy * 0.02 * strength, -18, 18)
    );
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rArm' ? 'lShoulder' : 'rShoulder',
      clampValue(-side * dx * 0.018 * strength, -18, 18)
    );
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rArm' ? 'rHipGirdle' : 'lHipGirdle',
      clampValue(side * dx * 0.012 * strength, -5, 5)
    );
    nextPose.root = {
      x: nextPose.root.x + dx * 0.03 * strength,
      y: nextPose.root.y + Math.max(-18, Math.min(18, dy * 0.015 * strength)),
    };
  } else {
    applyDeltaToPoseKey(nextPose, 'waist', clampValue(side * dx * 0.032 * strength, -26, 26));
    applyDeltaToPoseKey(nextPose, 'torso', clampValue(side * dx * 0.018 * strength - dy * 0.008 * strength, -18, 18));
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rLeg' ? 'rHipGirdle' : 'lHipGirdle',
      clampValue(side * dx * 0.045 * strength - dy * 0.02 * strength, -5, 5)
    );
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rLeg' ? 'rThigh' : 'lThigh',
      clampValue(side * dx * 0.018 * strength - dy * 0.015 * strength, -24, 24)
    );
    applyDeltaToPoseKey(
      nextPose,
      limbName === 'rLeg' ? 'lShoulder' : 'rShoulder',
      clampValue(-side * dx * 0.02 * strength, -16, 16)
    );
    nextPose.root = {
      x: nextPose.root.x + dx * 0.12 * strength,
      y: nextPose.root.y + dy * 0.08 * strength,
    };
  }

  return nextPose;
};

const solveKneelPosture = (
  pose: Pose,
  originalPose: Pose,
  mode: KinematicMode,
  limbName: 'rLeg' | 'lLeg',
  kneePart: PartName.RSkin | PartName.LSkin,
  kneeTarget: Vector2D,
  activePins: AnchorName[],
  facingMode: FacingMode
): Pose => {
  const nextPose = { ...pose };
  const joints = getJointPositions(nextPose, activePins);
  const kneePos = joints[kneePart];
  if (!kneePos) return pose;

  const drop = Math.max(0, kneeTarget.y - kneePos.y);
  if (drop < ANATOMY.FOOT * 0.18) return pose;

  const gravityStrength = mode === 'fullBody' ? 1 : mode === 'reactive' ? 0.65 : 0.35;
  const side = limbName === 'rLeg' ? 1 : -1;
  const normalizedDrop = Math.min(1, drop / (ANATOMY.LEG_LOWER * 0.9));

  nextPose.root = {
    x: nextPose.root.x + (kneeTarget.x - kneePos.x) * 0.12 * gravityStrength,
    y: nextPose.root.y + drop * 0.34 * gravityStrength,
  };
  applyDeltaToPoseKey(nextPose, 'waist', side * 12 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, 'torso', -side * 8 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'rHipGirdle' : 'lHipGirdle', side * 5 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'rThigh' : 'lThigh', 10 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'rCalf' : 'lCalf', 18 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'lThigh' : 'rThigh', -6 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'lCalf' : 'rCalf', 10 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'lShoulder' : 'rShoulder', -10 * normalizedDrop * gravityStrength);
  applyDeltaToPoseKey(nextPose, limbName === 'rLeg' ? 'rShoulder' : 'lShoulder', 6 * normalizedDrop * gravityStrength);

  let solvedPose = solveFABRIK(nextPose, limbName, kneePart, kneeTarget, activePins, facingMode, 24, 0.35);

  const supportLeg = limbName === 'rLeg' ? 'lLeg' : 'rLeg';
  const supportAnkleTarget = getJointPositions(originalPose, activePins)[supportLeg === 'rLeg' ? PartName.RAnkle : PartName.LAnkle];
  if (supportAnkleTarget && mode !== 'fabrik') {
    solvedPose = solveFABRIK(
      solvedPose,
      supportLeg,
      supportLeg === 'rLeg' ? PartName.RAnkle : PartName.LAnkle,
      supportAnkleTarget,
      [],
      facingMode,
      mode === 'fullBody' ? 18 : 12,
      0.5
    );
  }

  if (mode === 'fullBody') {
    const oppositeKnee = limbName === 'rLeg' ? PartName.LSkin : PartName.RSkin;
    const oppositeKneePos = getJointPositions(solvedPose, activePins)[oppositeKnee];
    if (oppositeKneePos && kneeTarget.y - oppositeKneePos.y > ANATOMY.FOOT * 0.3) {
      const settleTarget = {
        x: oppositeKneePos.x + (kneeTarget.x - oppositeKneePos.x) * 0.25,
        y: oppositeKneePos.y + (kneeTarget.y - oppositeKneePos.y) * 0.45,
      };
      solvedPose = solveFABRIK(
        solvedPose,
        supportLeg,
        oppositeKnee,
        settleTarget,
        [],
        facingMode,
        18,
        0.45
      );
    }
  }

  return solvedPose;
};

export const solvePoseWithIKMode = (
  pose: Pose,
  mode: KinematicMode,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName,
  target: Vector2D,
  activePins: AnchorName[],
  facingMode: FacingMode,
  handModes: { left: HandMode; right: HandMode }
): Pose => {
  if ((effectorPart === PartName.RSkin || effectorPart === PartName.LSkin) && mode !== 'fk') {
    let kneelPose = solveKneelPosture(
      pose,
      pose,
      mode,
      limbName as 'rLeg' | 'lLeg',
      effectorPart as PartName.RSkin | PartName.LSkin,
      target,
      activePins,
      facingMode
    );
    kneelPose = preservePinnedAnchors(kneelPose, pose, activePins, limbName, facingMode, mode === 'fullBody' ? 2 : 1);
    return kneelPose;
  }

  let solvedPose =
    mode === 'fullBody'
      ? solveFABRIK(pose, limbName, effectorPart, target, activePins, facingMode, 24, 0.35)
      : solveFABRIK(pose, limbName, effectorPart, target, activePins, facingMode);

  if (mode === 'reactive' || mode === 'fullBody') {
    const strength = mode === 'fullBody' ? 1.1 : 0.7;
    solvedPose = applyWholeBodyCompensation(solvedPose, pose, limbName, effectorPart, activePins, strength);
    solvedPose = solveFABRIK(
      solvedPose,
      limbName,
      effectorPart,
      target,
      activePins,
      facingMode,
      mode === 'fullBody' ? 26 : 18,
      0.4
    );
    solvedPose = preservePinnedAnchors(solvedPose, pose, activePins, limbName, facingMode, mode === 'fullBody' ? 2 : 1);
    if (mode === 'fullBody') {
      solvedPose = solveFABRIK(solvedPose, limbName, effectorPart, target, activePins, facingMode, 20, 0.35);
    }
  }

  if (limbName === 'lLeg') {
    solvedPose = solveGroundAwareAnkle(solvedPose, false, activePins, FLOOR_HEIGHT);
  } else if (limbName === 'rLeg') {
    solvedPose = solveGroundAwareAnkle(solvedPose, true, activePins, FLOOR_HEIGHT);
  }

  if (limbName === 'lArm') {
    solvedPose = applyHandModeBias(solvedPose, false, handModes.left, activePins);
  } else if (limbName === 'rArm') {
    solvedPose = applyHandModeBias(solvedPose, true, handModes.right, activePins);
  }

  return solvedPose;
};

export const solveGroundAwareAnkle = (
  pose: Pose,
  isRight: boolean,
  activePins: AnchorName[],
  floorY: number
): Pose => {
  const ankleKey = isRight ? 'rAnkle' : 'lAnkle';
  const anklePart = isRight ? PartName.RAnkle : PartName.LAnkle;
  const kneePart = isRight ? PartName.RSkin : PartName.LSkin;
  const ankleBaseRot = BASE_ROTATIONS[ankleKey as keyof typeof BASE_ROTATIONS] || 0;

  const joints = getJointPositions(pose, activePins);
  const anklePos = joints[anklePart];
  const kneePos = joints[kneePart];
  if (!anklePos || !kneePos) return pose;

  const groundProximity = floorY - anklePos.y;
  if (groundProximity > ANATOMY.FOOT * 1.5) return pose;

  const calfGlobalDeg = deg(Math.atan2(kneePos.x - anklePos.x, anklePos.y - kneePos.y));
  const cosTarget = (floorY - anklePos.y) / ANATOMY.FOOT;
  if (Math.abs(cosTarget) > 1) return pose;

  const acos1 = deg(Math.acos(cosTarget));
  const acos2 = -acos1;
  const currentLocal = (pose as any)[ankleKey] || 0;
  const currentWorldAngle = calfGlobalDeg + ankleBaseRot + currentLocal;
  const diff1 = Math.abs(normalizeAngleDeg(acos1 - currentWorldAngle));
  const diff2 = Math.abs(normalizeAngleDeg(acos2 - currentWorldAngle));
  const targetWorldAngle = diff1 <= diff2 ? acos1 : acos2;

  let targetLocal = normalizeAngleDeg(targetWorldAngle - calfGlobalDeg - ankleBaseRot);
  const limits = JOINT_LIMITS[ankleKey];
  if (limits) {
    targetLocal = Math.max(limits.min, Math.min(limits.max, targetLocal));
  }

  const t = Math.max(0, 1 - groundProximity / (ANATOMY.FOOT * 1.5));
  const blendedLocal = lerp(currentLocal, targetLocal, t);

  const newPose = { ...pose };
  (newPose as any)[ankleKey] = blendedLocal;
  return newPose;
};

export const applyHandModeBias = (
  pose: Pose,
  isRight: boolean,
  mode: HandMode,
  activePins: AnchorName[]
): Pose => {
  if (mode === 'neutral') return pose;

  const wristKey = isRight ? 'rWrist' : 'lWrist';
  const forearmKey = isRight ? 'rForearm' : 'lForearm';
  const elbowPart = isRight ? PartName.RElbow : PartName.LElbow;
  const wristPart = isRight ? PartName.RWrist : PartName.LWrist;

  const joints = getJointPositions(pose, activePins);
  const elbowPos = joints[elbowPart];
  const wristPos = joints[wristPart];
  if (!elbowPos || !wristPos) return pose;

  const forearmWorldAngle = deg(Math.atan2(elbowPos.x - wristPos.x, wristPos.y - elbowPos.y));
  const horizontalFactor = Math.abs(Math.sin(rad(forearmWorldAngle)));
  const baseDroop = mode === 'passive' ? 18 : 38;
  const adjustedDroop = baseDroop * (0.35 + 0.65 * horizontalFactor);

  const newPose = { ...pose };
  const currentWrist = (newPose as any)[wristKey] || 0;
  let newWrist = currentWrist + adjustedDroop;
  const wristLimits = JOINT_LIMITS[wristKey];
  if (wristLimits) {
    newWrist = Math.max(wristLimits.min, Math.min(wristLimits.max, newWrist));
  }
  (newPose as any)[wristKey] = newWrist;

  if (mode === 'heavy') {
    const currentForearm = (newPose as any)[forearmKey] || 0;
    const sag = 12 * horizontalFactor;
    let newForearm = currentForearm + sag;
    const forearmLimits = JOINT_LIMITS[forearmKey];
    if (forearmLimits) {
      newForearm = Math.max(forearmLimits.min, Math.min(forearmLimits.max, newForearm));
    }
    (newPose as any)[forearmKey] = newForearm;
  }

  return newPose;
};

export const solveHulaWaist = (
  basePose: Pose,
  dx: number,
  dy: number,
  activePins: AnchorName[],
  facingMode: FacingMode
): Pose => {
  const lateralScale = 0.45;
  const swayScale = 0.028;
  const crouchScale = 0.45;
  const torsoCounter = 0.5;
  const armCounter = 0.7;

  const waistSway = Math.max(-42, Math.min(42, dx * swayScale));
  const torsoChange = -waistSway * torsoCounter;

  let nextPose: Pose = {
    ...basePose,
    root: {
      x: basePose.root.x + dx * lateralScale,
      y: basePose.root.y + dy * crouchScale,
    },
    waist: (basePose.waist || 0) + waistSway,
    torso: (basePose.torso || 0) + torsoChange,
  };

  const baseJoints = getJointPositions(basePose, activePins);
  const legPins: Array<{ pin: AnchorName; limb: 'lLeg' | 'rLeg'; effector: PartName }> = [
    { pin: PartName.LAnkle, limb: 'lLeg', effector: PartName.LAnkle },
    { pin: PartName.RAnkle, limb: 'rLeg', effector: PartName.RAnkle },
    { pin: 'lFootTip', limb: 'lLeg', effector: PartName.LAnkle },
    { pin: 'rFootTip', limb: 'rLeg', effector: PartName.RAnkle },
  ];

  for (const { pin, limb, effector } of legPins) {
    if (!activePins.includes(pin)) continue;
    const target = baseJoints[pin as string];
    if (!target) continue;
    nextPose = solveFABRIK(nextPose, limb, effector, target, [], facingMode);
    nextPose = solveGroundAwareAnkle(nextPose, limb === 'rLeg', [], target.y + 1);
  }

  const netSpine = waistSway + torsoChange;
  const armAdjust = -netSpine * armCounter;
  nextPose = {
    ...nextPose,
    lShoulder: (basePose.lShoulder || 0) + armAdjust,
    rShoulder: (basePose.rShoulder || 0) + armAdjust,
  };

  return nextPose;
};

export const solveHeadAim = (
  pose: Pose,
  target: Vector2D,
  activePins: AnchorName[]
): Pose => {
  const joints = getJointPositions(pose, activePins);
  const chin = joints[PartName.Head];
  if (!chin) return pose;

  const torsoTopAngle = getTotalRotation('bodyRotation', pose) + getTotalRotation(PartName.Torso, pose);
  const desiredHeadGlobalAngle = deg(Math.atan2(target.y - chin.y, target.x - chin.x)) + 90;
  const desiredOffset = normalizeAngleDeg(desiredHeadGlobalAngle - torsoTopAngle);

  const nextPose = { ...pose };
  const collarLimits = JOINT_LIMITS[PartName.Collar];
  const headLimits = JOINT_LIMITS[PartName.Head];

  let collarAngle = desiredOffset * 0.4;
  if (collarLimits) {
    collarAngle = Math.max(collarLimits.min, Math.min(collarLimits.max, collarAngle));
  }

  let headAngle = desiredOffset - collarAngle;
  if (headLimits) {
    headAngle = Math.max(headLimits.min, Math.min(headLimits.max, headAngle));
  }

  if (headLimits && desiredOffset - collarAngle !== headAngle) {
    const remaining = desiredOffset - headAngle;
    if (collarLimits) {
      collarAngle = Math.max(collarLimits.min, Math.min(collarLimits.max, remaining));
    } else {
      collarAngle = remaining;
    }
  }

  nextPose.collar = collarAngle;
  nextPose.head = headAngle;
  return nextPose;
};

export const solveChinPull = (
  pose: Pose,
  target: Vector2D,
  activePins: AnchorName[]
): Pose => {
  const joints = getJointPositions(pose, activePins);
  const vJoint = joints[PartName.Collar];
  const chin = joints[PartName.Head];
  if (!vJoint || !chin) return pose;

  const torsoTopAngle = getTotalRotation('bodyRotation', pose) + getTotalRotation(PartName.Torso, pose);
  const desiredNeckGlobalAngle = deg(Math.atan2(target.y - vJoint.y, target.x - vJoint.x)) + 90;
  const desiredOffset = normalizeAngleDeg(desiredNeckGlobalAngle - torsoTopAngle);

  const nextPose = { ...pose };
  const collarLimits = JOINT_LIMITS[PartName.Collar];
  const headLimits = JOINT_LIMITS[PartName.Head];
  const currentHeadGlobalAngle = deg(Math.atan2(joints.headTip.y - chin.y, joints.headTip.x - chin.x)) + 90;
  const desiredHeadLocal = normalizeAngleDeg(currentHeadGlobalAngle - torsoTopAngle - desiredOffset);

  let collarAngle = desiredOffset * 0.8;
  if (collarLimits) {
    collarAngle = Math.max(collarLimits.min, Math.min(collarLimits.max, collarAngle));
  }

  let headAngle = desiredHeadLocal * 0.7;
  if (headLimits) {
    headAngle = Math.max(headLimits.min, Math.min(headLimits.max, headAngle));
  }

  nextPose.collar = collarAngle;
  nextPose.head = headAngle;
  return nextPose;
};

/**
 * Mirrors a pose left-to-right.
 */
export const mirrorPose = (pose: Pose): Pose => {
  const newPose = { ...pose };
  
  // Swap left/right values
  const swap = (keyL: keyof Pose, keyR: keyof Pose) => {
    const temp = newPose[keyL];
    (newPose as any)[keyL] = newPose[keyR];
    (newPose as any)[keyR] = temp;
  };

  // Negate angles for mirrored parts
  const flip = (key: keyof Pose) => {
    if (typeof newPose[key] === 'number') {
      (newPose as any)[key] = -(newPose[key] as number);
    }
  };

  swap('lShoulder', 'rShoulder');
  swap('lForearm', 'rForearm');
  swap('lWrist', 'rWrist');
  swap('lThigh', 'rThigh');
  swap('lCalf', 'rCalf');
  swap('lAnkle', 'rAnkle');
  swap('lToe', 'rToe');
  swap('lShoulderGirdle', 'rShoulderGirdle');
  swap('lHipGirdle', 'rHipGirdle');

  flip('lShoulder'); flip('rShoulder');
  flip('lForearm'); flip('rForearm');
  flip('lWrist'); flip('rWrist');
  flip('lThigh'); flip('rThigh');
  flip('lCalf'); flip('rCalf');
  flip('lAnkle'); flip('rAnkle');
  flip('lToe'); flip('rToe');
  flip('lShoulderGirdle'); flip('rShoulderGirdle');
  flip('lHipGirdle'); flip('rHipGirdle');

  // Torso/Waist/Collar/Head negate rotation
  flip('torso'); flip('waist'); flip('collar'); flip('head');
  
  // Body rotation flip
  newPose.bodyRotation = -newPose.bodyRotation;
  
  // Root X flip
  newPose.root = { ...newPose.root, x: -newPose.root.x };
  if (newPose.vOffset) {
    newPose.vOffset = { ...newPose.vOffset, x: -newPose.vOffset.x };
  }

  return newPose;
};

/**
 * Basic IK Solver using FABRIK.
 */
export const solveIK = (
  pose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName,
  target: Vector2D,
  activePins: AnchorName[]
): Pose => {
  return solveFABRIK(pose, limbName, effectorPart, target, activePins);
};

/**
 * Advanced IK Solver combining FABRIK with constraints.
 */
export const solveAdvancedIK = (
  pose: Pose,
  limbName: 'rArm' | 'lArm' | 'rLeg' | 'lLeg',
  effectorPart: PartName,
  target: Vector2D,
  activePins: AnchorName[]
): Pose => {
  const coarsePose = solveFABRIK(pose, limbName, effectorPart, target, activePins, 'front', 12, 1);
  return solveFABRIK(coarsePose, limbName, effectorPart, target, activePins, 'front', 24, 0.5);
};

/**
 * Applies soft physics corrections to a pose.
 */
export const applyPhysicsCorrections = (
  pose: Pose,
  activePins: AnchorName[],
  isGroundEnabled: boolean,
  floorHeight: number,
  draggedPart: PartName | null,
  dragDelta: Vector2D,
  faceControls: FaceControls = DEFAULT_FACE_CONTROLS
): Pose => {
  let correctedPose = { ...pose };
  let joints = getJointPositions(correctedPose, activePins);

  if (isGroundEnabled) {
    correctedPose = applyGroundContact(correctedPose, joints, floorHeight);
    joints = getJointPositions(correctedPose, activePins);
  }

  correctedPose = applySpringFeel(correctedPose, joints, activePins);
  joints = getJointPositions(correctedPose, activePins);

  correctedPose = applyCoMStabilizer(correctedPose, joints, activePins);
  joints = getJointPositions(correctedPose, activePins);

  if (draggedPart && (draggedPart.includes('Hand') || draggedPart.includes('Wrist') || draggedPart.includes('Shoulder'))) {
    const ARM_WEIGHT = 0.15;
    correctedPose.bodyRotation -= dragDelta.x * ARM_WEIGHT * 0.05;
  }

  const skullAngle = correctedPose.bodyRotation + correctedPose.torso + correctedPose.waist + correctedPose.collar + correctedPose.head;
  const turnT = Math.sin(skullAngle * 0.85);
  const faceShift = getFaceShift(faceControls, skullAngle, turnT);

  if (!faceControls.locked && faceControls.rotationEnabled) {
    correctedPose.head += faceShift.offset;
  }

  correctedPose.vOffset = correctedPose.vOffset || { x: 0, y: 0 };
  if (faceShift.eyeShift) {
    correctedPose.vOffset = { ...correctedPose.vOffset, x: correctedPose.vOffset.x + faceShift.eyeShift };
  }

  return correctedPose;
};
