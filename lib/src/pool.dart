import 'dart:math' as math;

import 'math.dart';
import 'projector.dart';

/// Pooled decal instance tracked by [DecalPoolManager].
final class DecalPoolEntry {
  final int id;
  final String tag;
  final DecalVec3 position;
  final DecalVec3 normal;
  final DecalQuat rotation;
  final DecalVec3 size;
  final DecalMesh mesh;
  final double lifetimeSeconds;
  final double fadeDurationSeconds;
  double ageSeconds = 0.0;

  DecalPoolEntry({
    required this.id,
    required this.tag,
    required this.position,
    required this.normal,
    required this.rotation,
    required this.size,
    required this.mesh,
    required this.lifetimeSeconds,
    required this.fadeDurationSeconds,
  });

  double get opacity {
    if (lifetimeSeconds <= 0.0) return 1.0;
    final remaining = lifetimeSeconds - ageSeconds;
    if (remaining <= 0.0) return 0.0;
    if (fadeDurationSeconds <= 1e-5) return 1.0;
    return (remaining / fadeDurationSeconds).clamp(0.0, 1.0);
  }

  bool get isExpired => lifetimeSeconds > 0.0 && ageSeconds >= lifetimeSeconds;
}

/// Snapshot telemetry reported by [DecalPoolManager.stats].
final class DecalPoolStats {
  final int activeCount;
  final int maxCapacity;
  final int evictedCount;
  final int mergeCulledCount;
  final int totalTriangles;

  const DecalPoolStats({
    required this.activeCount,
    required this.maxCapacity,
    required this.evictedCount,
    required this.mergeCulledCount,
    required this.totalTriangles,
  });
}

/// Bounded ring-buffer decal manager with FIFO eviction, lifetime fadeout & proximity duplicate culling.
final class DecalPoolManager {
  final int maxCapacity;
  final double mergeCullRadius;
  final double defaultLifetimeSeconds;
  final double defaultFadeDurationSeconds;

  final List<DecalPoolEntry> _entries = [];
  int _nextId = 1;
  int _evictedCount = 0;
  int _mergeCulledCount = 0;

  DecalPoolManager({
    this.maxCapacity = 64,
    this.mergeCullRadius = 0.08,
    this.defaultLifetimeSeconds = 20.0,
    this.defaultFadeDurationSeconds = 3.0,
  });

  List<DecalPoolEntry> get entries => List.unmodifiable(_entries);
  int get activeCount => _entries.length;
  int get evictedCount => _evictedCount;
  int get mergeCulledCount => _mergeCulledCount;

  DecalPoolStats get stats {
    var tris = 0;
    for (final e in _entries) {
      tris += e.mesh.triangleCount;
    }
    return DecalPoolStats(
      activeCount: _entries.length,
      maxCapacity: maxCapacity,
      evictedCount: _evictedCount,
      mergeCulledCount: _mergeCulledCount,
      totalTriangles: tris,
    );
  }

  /// Registers a new decal in the ring buffer (or returns `null` when culled by [mergeCullRadius]).
  DecalPoolEntry? spawn({
    String tag = 'scar',
    required DecalVec3 position,
    required DecalVec3 normal,
    DecalQuat rotation = DecalQuat.identity,
    DecalVec3 size = const DecalVec3(1.0, 1.0, 0.4),
    DecalMesh mesh = DecalMesh.empty,
    double? lifetimeSeconds,
    double? fadeDurationSeconds,
    bool bypassMergeCull = false,
  }) {
    final unitN = normal.normalized;
    if (!bypassMergeCull && mergeCullRadius > 1e-6) {
      final radSq = mergeCullRadius * mergeCullRadius;
      for (final existing in _entries) {
        if ((existing.position - position).lengthSquared <= radSq &&
            existing.normal.dot(unitN) >= 0.85) {
          _mergeCulledCount++;
          return null;
        }
      }
    }

    final cap = math.max(1, maxCapacity);
    while (_entries.length >= cap) {
      _entries.removeAt(0);
      _evictedCount++;
    }

    final entry = DecalPoolEntry(
      id: _nextId++,
      tag: tag,
      position: position,
      normal: unitN,
      rotation: rotation,
      size: size,
      mesh: mesh,
      lifetimeSeconds: lifetimeSeconds ?? defaultLifetimeSeconds,
      fadeDurationSeconds: fadeDurationSeconds ?? defaultFadeDurationSeconds,
    );
    _entries.add(entry);
    return entry;
  }

  /// Advances ages of all pooled decals by [dt] seconds and removes expired entries.
  int tick(double dt) {
    if (dt <= 0.0 || _entries.isEmpty) return 0;
    var removed = 0;
    for (var i = _entries.length - 1; i >= 0; i--) {
      final e = _entries[i];
      e.ageSeconds += dt;
      if (e.isExpired) {
        _entries.removeAt(i);
        removed++;
      }
    }
    return removed;
  }

  void clear() {
    _entries.clear();
  }
}
