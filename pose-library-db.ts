
// This file contains a static database of predefined poses for the mannequin.
// Each pose is represented as an object with an ID, category, name, source, and pose data.
// The pose data is stored as a compact string, which can be parsed into a Pose object.

// Note: The pose data strings have been updated to reflect the new part naming
// (e.g., 'w' for waist, 'c' for collar, 'lw'/'rw' for wrist/hand, 'la'/'ra' for ankle/foot),
// and the new proportional sizing and floor height.

import { SCALE_FACTOR } from './constants'; // Import SCALE_FACTOR
import { mirrorPose } from './utils/pose-mirror'; // Import mirrorPose

const scaleRoot = (rootString: string): string => {
  const parts = rootString.split(':');
  if (parts.length === 2 && parts[0] === 'r') {
    const [xStr, yStr] = parts[1].split(',');
    const x = parseFloat(xStr) * SCALE_FACTOR;
    const y = parseFloat(yStr) * SCALE_FACTOR;
    return `r:${x},${y}`;
  }
  return rootString;
};

const originalPoses = [
  // Updated T-Pose data string for new T_POSE_ROOT_Y
  { "id": "B01", "cat": "Base", "name": "T-Pose", "src": "Bitruvius", "data": scaleRoot("r:0,600") + ";br:0;w:0;t:0;c:0;h:0;ls:-90;le:0;lw:0;rs:90;re:0;rw:0;lt:0;lc:0;la:0;rt:0;rc:0;ra:0" },
  { "id": "B02", "cat": "Base", "name": "Death Pose", "src": "Bitruvius", "data": "r:-19.77,2498.36;br:2;w:-5.53;t:2.05;c:0.3;h:0.1;rs:101.5;re:155.08;rw:65.75;ls:-105.35;le:-149.93;lw:-80.42;rt:-17.99;rc:-0.01;ra:-56.12;lt:-4.88;lc:0.01;la:148.27" },
];

// Poses identified as "left-leaning" for mirroring
const posesToMirrorInfo = [
  { id: "A06", name: "Shield", cat: "Action" },
  { id: "D02", name: "Ballerina", cat: "Dance" },
  { id: "S01", name: "Tree Ornament", cat: "Still" },
];

const generatedMirroredPoses = posesToMirrorInfo.map(pInfo => {
  const originalPose = originalPoses.find(pose => pose.id === pInfo.id);
  if (!originalPose) {
    console.warn(`Original pose with ID ${pInfo.id} not found for mirroring.`);
    return null;
  }
  return mirrorPose(originalPose.data, pInfo.id, pInfo.name, pInfo.cat);
}).filter(Boolean); // Filter out any null entries if original pose wasn't found

export const POSE_LIBRARY_DB = [
  ...originalPoses,
  ...(generatedMirroredPoses as any), // Cast to any to safely spread (after filtering nulls)
];
