import { AnchorName, PartName, Pose, SubdivisionInterval, SubdivisionNode, SubdivisionSuggestion, Vector2D } from '../types';
import { getJointPositions, interpolatePoses } from './kinematics';

const EPSILON = 0.0001;
const FOOT_PIN_DISTANCE_THRESHOLD = 18;

const clampProgress = (value: number) => Math.max(0, Math.min(1, value));
const progressKey = (value: number) => clampProgress(value).toFixed(4);
const createSuggestionId = (type: SubdivisionSuggestion['type'], progress: number) => `${type}-${progressKey(progress)}`;

const getDistance = (a: Vector2D, b: Vector2D) => Math.hypot(a.x - b.x, a.y - b.y);

const getTorsoVector = (pose: Pose) => {
  const joints = getJointPositions(pose, []);
  const root = joints.root;
  const collar = joints[PartName.Collar];
  if (!root || !collar) {
    return { x: 0, y: -1 };
  }
  return {
    x: collar.x - root.x,
    y: collar.y - root.y,
  };
};

const getVerticality = (pose: Pose) => {
  const vector = getTorsoVector(pose);
  const magnitude = Math.hypot(vector.x, vector.y) || 1;
  return Math.abs(vector.y) / magnitude;
};

const hasUpperLimbPins = (pins: AnchorName[]) =>
  pins.some(pin => pin === PartName.Head || pin === PartName.Collar || pin === 'lHandTip' || pin === 'rHandTip' || pin === PartName.LWrist || pin === PartName.RWrist);

export const sortSubdivisionNodes = (nodes: SubdivisionNode[]) =>
  [...nodes].sort((a, b) => a.progress - b.progress);

export const getAllSubdivisionStops = (
  startPose: Pose | null,
  endPose: Pose | null,
  nodes: SubdivisionNode[],
): Array<{ progress: number; pose: Pose; label: string; nodeId: string | null }> => {
  const authored = sortSubdivisionNodes(nodes).map(node => ({
    progress: node.progress,
    pose: node.pose,
    label: node.label,
    nodeId: node.id,
  }));
  const stops: Array<{ progress: number; pose: Pose; label: string; nodeId: string | null }> = [];
  if (startPose) {
    stops.push({ progress: 0, pose: startPose, label: 'Start', nodeId: null });
  }
  stops.push(...authored);
  if (endPose) {
    stops.push({ progress: 1, pose: endPose, label: 'End', nodeId: null });
  }
  return stops.sort((a, b) => a.progress - b.progress);
};

export const getAvailableSubdivisionIntervals = (
  startPose: Pose | null,
  endPose: Pose | null,
  nodes: SubdivisionNode[],
): SubdivisionInterval[] => {
  const stops = getAllSubdivisionStops(startPose, endPose, nodes);
  const intervals: SubdivisionInterval[] = [];
  for (let i = 0; i < stops.length - 1; i += 1) {
    const start = stops[i];
    const end = stops[i + 1];
    if (end.progress - start.progress > EPSILON) {
      intervals.push({ startProgress: start.progress, endProgress: end.progress });
    }
  }
  return intervals;
};

export const getIntervalMidpointProgress = (interval: SubdivisionInterval) =>
  clampProgress((interval.startProgress + interval.endProgress) / 2);

export const formatSubdivisionProgress = (progress: number) => `F${Math.round(clampProgress(progress) * 60)}`;

export const buildSubdivisionNodeLabel = (progress: number) => {
  if (Math.abs(progress) < EPSILON) return 'Start';
  if (Math.abs(progress - 1) < EPSILON) return 'End';
  return `Slice ${formatSubdivisionProgress(progress)}`;
};

export const sampleSubdivisionPose = (
  startPose: Pose | null,
  endPose: Pose | null,
  nodes: SubdivisionNode[],
  progress: number,
): Pose | null => {
  if (!startPose || !endPose) return null;
  const target = clampProgress(progress);
  if (Math.abs(target) < EPSILON) return startPose;
  if (Math.abs(target - 1) < EPSILON) return endPose;

  const stops = getAllSubdivisionStops(startPose, endPose, nodes);
  const exact = stops.find(stop => Math.abs(stop.progress - target) < EPSILON);
  if (exact) return exact.pose;

  const left = [...stops].reverse().find(stop => stop.progress < target);
  const right = stops.find(stop => stop.progress > target);
  if (!left || !right) return null;
  const span = Math.max(right.progress - left.progress, EPSILON);
  const t = (target - left.progress) / span;
  return interpolatePoses(left.pose, right.pose, t);
};

export const inferSubdivisionSuggestions = (
  startPose: Pose,
  endPose: Pose,
  progress: number,
  activePins: AnchorName[],
): SubdivisionSuggestion[] => {
  const startJoints = getJointPositions(startPose, []);
  const endJoints = getJointPositions(endPose, []);
  const suggestions: SubdivisionSuggestion[] = [];

  const leftAnkleStart = startJoints[PartName.LAnkle];
  const leftAnkleEnd = endJoints[PartName.LAnkle];
  const rightAnkleStart = startJoints[PartName.RAnkle];
  const rightAnkleEnd = endJoints[PartName.RAnkle];
  const leftStable = leftAnkleStart && leftAnkleEnd && getDistance(leftAnkleStart, leftAnkleEnd) <= FOOT_PIN_DISTANCE_THRESHOLD;
  const rightStable = rightAnkleStart && rightAnkleEnd && getDistance(rightAnkleStart, rightAnkleEnd) <= FOOT_PIN_DISTANCE_THRESHOLD;
  if (leftStable || rightStable) {
    suggestions.push({
      id: createSuggestionId('autoPinFeet', progress),
      type: 'autoPinFeet',
      label: 'Auto-Pin Feet',
      description: leftStable && rightStable
        ? 'Both ankles hold their world position. Treat the feet as likely floor pins.'
        : 'One ankle holds its world position. Consider pinning the planted foot for the slice.',
      active: true,
      applied: false,
    });
  }

  const startVerticality = getVerticality(startPose);
  const endVerticality = getVerticality(endPose);
  if (startVerticality < 0.55 && endVerticality > 0.84) {
    suggestions.push({
      id: createSuggestionId('torsoWidthMorph', progress),
      type: 'torsoWidthMorph',
      label: 'Torso Width Morph',
      description: 'The torso shifts from horizontal toward vertical. Flag this slice for width expansion metadata.',
      active: true,
      applied: false,
    });
  }

  const armTravel =
    Math.abs(endPose.lShoulder - startPose.lShoulder) +
    Math.abs(endPose.rShoulder - startPose.rShoulder) +
    Math.abs(endPose.lForearm - startPose.lForearm) +
    Math.abs(endPose.rForearm - startPose.rForearm);
  if (!hasUpperLimbPins(activePins) && armTravel > 28) {
    suggestions.push({
      id: createSuggestionId('armGravityHint', progress),
      type: 'armGravityHint',
      label: 'Arm Gravity / Drag',
      description: 'The upper limbs are unpinned across this move. Keep a gravity/drag hint attached while shaping the ghost.',
      active: true,
      applied: false,
    });
  }

  return suggestions;
};

export const createSubdivisionGhost = (
  interval: SubdivisionInterval,
  startPose: Pose,
  endPose: Pose,
  activePins: AnchorName[],
) => {
  const progress = getIntervalMidpointProgress(interval);
  return {
    interval,
    progress,
    pose: interpolatePoses(startPose, endPose, 0.5),
    suggestions: inferSubdivisionSuggestions(startPose, endPose, progress, activePins),
    label: buildSubdivisionNodeLabel(progress),
    editing: false,
  };
};
