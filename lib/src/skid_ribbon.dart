import 'dart:math' as math;

import 'math.dart';
import 'projector.dart';

/// Single stamp point inside a continuous tire skid / footprint ribbon strip.
final class SkidSamplePoint {
  final DecalVec3 position;
  final DecalVec3 normal;
  final double intensity;
  final double width;
  final double timestamp;
  final bool startsNewStrip;

  const SkidSamplePoint({
    required this.position,
    required this.normal,
    required this.intensity,
    required this.width,
    required this.timestamp,
    this.startsNewStrip = false,
  });
}

/// Builds continuous ground-aligned quad-strip meshes for tire tracks, skids, and footprints.
final class SkidRibbonBuilder {
  final double defaultWidth;
  final double minSegmentDistance;
  final double maxSegmentDistance;
  final double surfaceOffset;
  final double lifetimeSeconds;
  final double fadeDurationSeconds;
  final int maxSamples;

  final List<SkidSamplePoint> _samples = [];
  double _elapsedTime = 0.0;
  bool _pendingBreak = true;

  SkidRibbonBuilder({
    this.defaultWidth = 0.28,
    this.minSegmentDistance = 0.16,
    this.maxSegmentDistance = 2.5,
    this.surfaceOffset = 0.012,
    this.lifetimeSeconds = 12.0,
    this.fadeDurationSeconds = 2.5,
    this.maxSamples = 256,
  });

  List<SkidSamplePoint> get samples => List.unmodifiable(_samples);
  int get sampleCount => _samples.length;
  double get elapsedTime => _elapsedTime;

  /// Marks a discontinuity so the next added sample starts a fresh strip instead of bridging across gaps.
  void breakStrip() {
    _pendingBreak = true;
  }

  /// Appends a world-space contact stamp if it moved far enough beyond [minSegmentDistance].
  bool addSample({
    required DecalVec3 position,
    DecalVec3 normal = DecalVec3.unitY,
    double intensity = 1.0,
    double? width,
    bool forceBreak = false,
  }) {
    final unitN = normal.normalized;
    final resolvedWidth = math.max(0.02, width ?? defaultWidth);
    final clampedInt = intensity.clamp(0.0, 1.0);

    var startsNew = _pendingBreak || forceBreak || _samples.isEmpty;
    if (!startsNew && _samples.isNotEmpty) {
      final prev = _samples.last;
      final dist = position.distanceTo(prev.position);
      if (dist < minSegmentDistance) return false;
      if (dist > maxSegmentDistance) startsNew = true;
    }

    _pendingBreak = false;
    _samples.add(
      SkidSamplePoint(
        position: position,
        normal: unitN,
        intensity: clampedInt,
        width: resolvedWidth,
        timestamp: _elapsedTime,
        startsNewStrip: startsNew,
      ),
    );

    if (maxSamples > 0 && _samples.length > maxSamples) {
      final overflow = _samples.length - maxSamples;
      _samples.removeRange(0, overflow);
      if (_samples.isNotEmpty && !_samples.first.startsNewStrip) {
        final s = _samples.first;
        _samples[0] = SkidSamplePoint(
          position: s.position,
          normal: s.normal,
          intensity: s.intensity,
          width: s.width,
          timestamp: s.timestamp,
          startsNewStrip: true,
        );
      }
    }
    return true;
  }

  /// Advances internal clock by [dt] seconds and prunes expired samples.
  void advance(double dt) {
    if (dt <= 0.0) return;
    _elapsedTime += dt;
    if (lifetimeSeconds <= 0.0 || _samples.isEmpty) return;

    var dropCount = 0;
    while (dropCount < _samples.length &&
        (_elapsedTime - _samples[dropCount].timestamp) >= lifetimeSeconds) {
      dropCount++;
    }
    if (dropCount > 0) {
      _samples.removeRange(0, dropCount);
      if (_samples.isNotEmpty && !_samples.first.startsNewStrip) {
        final s = _samples.first;
        _samples[0] = SkidSamplePoint(
          position: s.position,
          normal: s.normal,
          intensity: s.intensity,
          width: s.width,
          timestamp: s.timestamp,
          startsNewStrip: true,
        );
      }
    }
  }

  void clear() {
    _samples.clear();
    _pendingBreak = true;
  }

  /// Triangulates all active sub-strips into a combined [DecalMesh] with smooth fade alpha & arc UVs.
  DecalMesh buildMesh() {
    if (_samples.length < 2) return DecalMesh.empty;

    final outVerts = <DecalVertex>[];
    final outIdx = <int>[];
    var minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;

    var runStart = 0;
    while (runStart < _samples.length) {
      var runEnd = runStart + 1;
      while (runEnd < _samples.length && !_samples[runEnd].startsNewStrip) {
        runEnd++;
      }

      final runCount = runEnd - runStart;
      if (runCount >= 2) {
        final baseVertex = outVerts.length;
        var arcLen = 0.0;

        for (var i = 0; i < runCount; i++) {
          final curr = _samples[runStart + i];
          if (i > 0) {
            arcLen += curr.position.distanceTo(_samples[runStart + i - 1].position);
          }

          final DecalVec3 tangent;
          if (i == 0) {
            tangent = (_samples[runStart + 1].position - curr.position).normalized;
          } else if (i == runCount - 1) {
            tangent = (curr.position - _samples[runStart + i - 1].position).normalized;
          } else {
            tangent =
                (_samples[runStart + i + 1].position - _samples[runStart + i - 1].position)
                    .normalized;
          }

          var side = tangent.cross(curr.normal);
          if (side.lengthSquared <= 1e-8) {
            side = tangent.cross(DecalVec3.unitX);
            if (side.lengthSquared <= 1e-8) side = DecalVec3.unitZ;
          }
          side = side.normalized;

          final halfW = curr.width * 0.5;
          final lift = curr.normal * surfaceOffset;
          final leftP = curr.position - side * halfW + lift;
          final rightP = curr.position + side * halfW + lift;

          var lifeFade = 1.0;
          if (lifetimeSeconds > 0.0 && fadeDurationSeconds > 1e-5) {
            final age = _elapsedTime - curr.timestamp;
            final remaining = lifetimeSeconds - age;
            lifeFade = (remaining / fadeDurationSeconds).clamp(0.0, 1.0);
          }
          final endpointFade = (i == 0 || i == runCount - 1) ? 0.25 : 1.0;
          final alpha = curr.intensity * lifeFade * endpointFade;

          for (final p in [leftP, rightP]) {
            if (p.x < minX) minX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.z < minZ) minZ = p.z;
            if (p.x > maxX) maxX = p.x;
            if (p.y > maxY) maxY = p.y;
            if (p.z > maxZ) maxZ = p.z;
          }

          outVerts.add(
            DecalVertex(
              position: leftP,
              normal: curr.normal,
              uv: DecalVec2(0.0, arcLen),
              alpha: alpha,
            ),
          );
          outVerts.add(
            DecalVertex(
              position: rightP,
              normal: curr.normal,
              uv: DecalVec2(1.0, arcLen),
              alpha: alpha,
            ),
          );

          if (i < runCount - 1) {
            final v0 = baseVertex + i * 2;
            final v1 = v0 + 1;
            final v2 = v0 + 2;
            final v3 = v0 + 3;
            outIdx.addAll([v0, v2, v1, v1, v2, v3]);
          }
        }
      }

      runStart = runEnd;
    }

    if (outIdx.isEmpty) return DecalMesh.empty;
    return DecalMesh(
      vertices: outVerts,
      indices: outIdx,
      boundsMin: DecalVec3(minX, minY, minZ),
      boundsMax: DecalVec3(maxX, maxY, maxZ),
    );
  }
}
