import 'dart:math' as math;

import 'math.dart';

/// Vertex tuple emitted by [DecalBoxProjector] and [SkidRibbonBuilder].
final class DecalVertex {
  final DecalVec3 position;
  final DecalVec3 normal;
  final DecalVec2 uv;
  final double alpha;

  const DecalVertex({
    required this.position,
    required this.normal,
    required this.uv,
    this.alpha = 1.0,
  });
}

/// Indexed triangle mesh output for a projected decal or skid ribbon.
final class DecalMesh {
  final List<DecalVertex> vertices;
  final List<int> indices;
  final DecalVec3 boundsMin;
  final DecalVec3 boundsMax;

  const DecalMesh({
    required this.vertices,
    required this.indices,
    required this.boundsMin,
    required this.boundsMax,
  });

  static const DecalMesh empty = DecalMesh(
    vertices: [],
    indices: [],
    boundsMin: DecalVec3.zero,
    boundsMax: DecalVec3.zero,
  );

  int get vertexCount => vertices.length;
  int get triangleCount => indices.length ~/ 3;
  bool get isEmpty => indices.length < 3 || vertices.isEmpty;
}

/// Lightweight target geometry snapshot fed into [DecalBoxProjector.project].
final class DecalSourceMesh {
  final List<DecalVec3> positions;
  final List<DecalVec3> normals;
  final List<int> indices;

  const DecalSourceMesh({
    required this.positions,
    required this.normals,
    required this.indices,
  });

  /// Builds an axis-aligned box surface target mesh at [center] for testing and standalone projection.
  factory DecalSourceMesh.box({
    DecalVec3 size = const DecalVec3(2.0, 2.0, 2.0),
    DecalVec3 center = DecalVec3.zero,
  }) {
    final hx = size.x * 0.5;
    final hy = size.y * 0.5;
    final hz = size.z * 0.5;
    final pos = <DecalVec3>[];
    final norm = <DecalVec3>[];
    final idx = <int>[];

    void addQuad(DecalVec3 a, DecalVec3 b, DecalVec3 c, DecalVec3 d, DecalVec3 n) {
      final base = pos.length;
      pos.addAll([center + a, center + b, center + c, center + d]);
      norm.addAll([n, n, n, n]);
      idx.addAll([base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    addQuad(DecalVec3(-hx, -hy, hz), DecalVec3(hx, -hy, hz), DecalVec3(hx, hy, hz), DecalVec3(-hx, hy, hz), DecalVec3.unitZ);
    addQuad(DecalVec3(hx, -hy, -hz), DecalVec3(-hx, -hy, -hz), DecalVec3(-hx, hy, -hz), DecalVec3(hx, hy, -hz), -DecalVec3.unitZ);
    addQuad(DecalVec3(-hx, hy, hz), DecalVec3(hx, hy, hz), DecalVec3(hx, hy, -hz), DecalVec3(-hx, hy, -hz), DecalVec3.unitY);
    addQuad(DecalVec3(-hx, -hy, -hz), DecalVec3(hx, -hy, -hz), DecalVec3(hx, -hy, hz), DecalVec3(-hx, -hy, hz), -DecalVec3.unitY);
    addQuad(DecalVec3(hx, -hy, hz), DecalVec3(hx, -hy, -hz), DecalVec3(hx, hy, -hz), DecalVec3(hx, hy, hz), DecalVec3.unitX);
    addQuad(DecalVec3(-hx, -hy, -hz), DecalVec3(-hx, -hy, hz), DecalVec3(-hx, hy, hz), DecalVec3(-hx, hy, -hz), -DecalVec3.unitX);

    return DecalSourceMesh(positions: pos, normals: norm, indices: idx);
  }

  /// Builds a subdivided horizontal plane at `y = center.y` with normal `+Y`.
  factory DecalSourceMesh.plane({
    double width = 4.0,
    double depth = 4.0,
    int subdivisions = 2,
    DecalVec3 center = DecalVec3.zero,
  }) {
    final pos = <DecalVec3>[];
    final norm = <DecalVec3>[];
    final idx = <int>[];
    final cols = math.max(1, subdivisions) + 1;
    for (var z = 0; z < cols; z++) {
      final fz = z / (cols - 1) - 0.5;
      for (var x = 0; x < cols; x++) {
        final fx = x / (cols - 1) - 0.5;
        pos.add(DecalVec3(center.x + fx * width, center.y, center.z + fz * depth));
        norm.add(DecalVec3.unitY);
      }
    }
    for (var z = 0; z < cols - 1; z++) {
      for (var x = 0; x < cols - 1; x++) {
        final i0 = z * cols + x;
        idx.addAll([i0, i0 + cols, i0 + cols + 1, i0, i0 + cols + 1, i0 + 1]);
      }
    }
    return DecalSourceMesh(positions: pos, normals: norm, indices: idx);
  }
}

/// Oriented 3D decal projection volume (local `-Z` points along projection ray direction).
final class DecalBoxSpec {
  final DecalVec3 center;
  final DecalQuat rotation;
  final DecalVec3 size;
  final double maxNormalAngleDegrees;
  final double surfaceOffset;
  final double edgeFadeFraction;
  final DecalVec2 uvTile;
  final DecalVec2 uvOffset;

  const DecalBoxSpec({
    this.center = DecalVec3.zero,
    this.rotation = DecalQuat.identity,
    this.size = const DecalVec3(1.0, 1.0, 0.5),
    this.maxNormalAngleDegrees = 65.0,
    this.surfaceOffset = 0.008,
    this.edgeFadeFraction = 0.15,
    this.uvTile = DecalVec2.one,
    this.uvOffset = DecalVec2.zero,
  });
}

final class _LocalClipVert {
  final DecalVec3 localPos;
  final DecalVec3 localNormal;

  const _LocalClipVert(this.localPos, this.localNormal);

  static _LocalClipVert lerp(_LocalClipVert a, _LocalClipVert b, double t) =>
      _LocalClipVert(
        DecalVec3.lerp(a.localPos, b.localPos, t),
        DecalVec3.lerp(a.localNormal, b.localNormal, t).normalized,
      );
}

/// Projects oriented decal quads cleanly across target triangle meshes via 6-plane OBB clipping.
abstract final class DecalBoxProjector {
  /// Clips [target] triangles to [box], culling backfacing/steep surfaces and remapping `[0..1]` UVs.
  static DecalMesh project({
    required DecalSourceMesh target,
    required DecalBoxSpec box,
  }) {
    if (target.indices.length < 3 || target.positions.isEmpty) {
      return DecalMesh.empty;
    }

    final rot = box.rotation.normalized;
    final invRot = rot.conjugate;
    final hw = math.max(1e-4, box.size.x * 0.5);
    final hh = math.max(1e-4, box.size.y * 0.5);
    final hd = math.max(1e-4, box.size.z * 0.5);

    final angleRad = box.maxNormalAngleDegrees.clamp(0.0, 179.0) * math.pi / 180.0;
    final minNormalDot = math.cos(angleRad);
    // In decal local space, projection travels along `-Z`, so struck surfaces face `+Z`.
    const localTowardProjector = DecalVec3(0.0, 0.0, 1.0);

    final outVerts = <DecalVertex>[];
    final outIdx = <int>[];
    var minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;

    for (var i = 0; i + 2 < target.indices.length; i += 3) {
      final i0 = target.indices[i];
      final i1 = target.indices[i + 1];
      final i2 = target.indices[i + 2];
      if (i0 < 0 ||
          i1 < 0 ||
          i2 < 0 ||
          i0 >= target.positions.length ||
          i1 >= target.positions.length ||
          i2 >= target.positions.length) {
        continue;
      }

      final w0 = target.positions[i0];
      final w1 = target.positions[i1];
      final w2 = target.positions[i2];
      final geomCross = (w1 - w0).cross(w2 - w0);
      if (geomCross.lengthSquared <= 1e-12) continue;

      final l0 = invRot.rotate(w0 - box.center);
      final l1 = invRot.rotate(w1 - box.center);
      final l2 = invRot.rotate(w2 - box.center);

      // Quick AABB reject before normal & 6-plane clipping.
      if (math.max(l0.x, math.max(l1.x, l2.x)) < -hw ||
          math.min(l0.x, math.min(l1.x, l2.x)) > hw ||
          math.max(l0.y, math.max(l1.y, l2.y)) < -hh ||
          math.min(l0.y, math.min(l1.y, l2.y)) > hh ||
          math.max(l0.z, math.max(l1.z, l2.z)) < -hd ||
          math.min(l0.z, math.min(l1.z, l2.z)) > hd) {
        continue;
      }

      final faceNormalLocal = invRot.rotate(geomCross.normalized).normalized;
      if (faceNormalLocal.dot(localTowardProjector) < minNormalDot) continue;

      final ln0 = i0 < target.normals.length
          ? invRot.rotate(target.normals[i0]).normalized
          : faceNormalLocal;
      final ln1 = i1 < target.normals.length
          ? invRot.rotate(target.normals[i1]).normalized
          : faceNormalLocal;
      final ln2 = i2 < target.normals.length
          ? invRot.rotate(target.normals[i2]).normalized
          : faceNormalLocal;

      var poly = <_LocalClipVert>[
        _LocalClipVert(l0, ln0),
        _LocalClipVert(l1, ln1),
        _LocalClipVert(l2, ln2),
      ];

      poly = _clipPlane(poly, (p) => p.x + hw);
      if (poly.length < 3) continue;
      poly = _clipPlane(poly, (p) => hw - p.x);
      if (poly.length < 3) continue;
      poly = _clipPlane(poly, (p) => p.y + hh);
      if (poly.length < 3) continue;
      poly = _clipPlane(poly, (p) => hh - p.y);
      if (poly.length < 3) continue;
      poly = _clipPlane(poly, (p) => p.z + hd);
      if (poly.length < 3) continue;
      poly = _clipPlane(poly, (p) => hd - p.z);
      if (poly.length < 3) continue;

      final baseVertex = outVerts.length;
      for (final cv in poly) {
        final worldN = rot.rotate(cv.localNormal).normalized;
        final worldP = box.center + rot.rotate(cv.localPos) + worldN * box.surfaceOffset;
        final u = ((cv.localPos.x + hw) / (2.0 * hw)) * box.uvTile.x + box.uvOffset.x;
        final v = ((cv.localPos.y + hh) / (2.0 * hh)) * box.uvTile.y + box.uvOffset.y;

        var alpha = 1.0;
        if (box.edgeFadeFraction > 1e-5) {
          final distToZEdge = 1.0 - (cv.localPos.z.abs() / hd).clamp(0.0, 1.0);
          alpha = (distToZEdge / box.edgeFadeFraction).clamp(0.0, 1.0);
        }

        if (worldP.x < minX) minX = worldP.x;
        if (worldP.y < minY) minY = worldP.y;
        if (worldP.z < minZ) minZ = worldP.z;
        if (worldP.x > maxX) maxX = worldP.x;
        if (worldP.y > maxY) maxY = worldP.y;
        if (worldP.z > maxZ) maxZ = worldP.z;

        outVerts.add(
          DecalVertex(
            position: worldP,
            normal: worldN,
            uv: DecalVec2(u, v),
            alpha: alpha,
          ),
        );
      }

      for (var t = 1; t + 1 < poly.length; t++) {
        outIdx.addAll([baseVertex, baseVertex + t, baseVertex + t + 1]);
      }
    }

    if (outIdx.isEmpty) return DecalMesh.empty;
    return DecalMesh(
      vertices: outVerts,
      indices: outIdx,
      boundsMin: DecalVec3(minX, minY, minZ),
      boundsMax: DecalVec3(maxX, maxY, maxZ),
    );
  }

  static List<_LocalClipVert> _clipPlane(
    List<_LocalClipVert> poly,
    double Function(DecalVec3 p) signedDistInside,
  ) {
    final result = <_LocalClipVert>[];
    for (var i = 0; i < poly.length; i++) {
      final j = (i + 1) % poly.length;
      final a = poly[i];
      final b = poly[j];
      final da = signedDistInside(a.localPos);
      final db = signedDistInside(b.localPos);
      final inA = da >= -1e-6;
      final inB = db >= -1e-6;

      if (inA && inB) {
        result.add(b);
      } else if (inA && !inB) {
        final denom = da - db;
        final t = denom.abs() > 1e-12 ? (da / denom).clamp(0.0, 1.0) : 0.0;
        result.add(_LocalClipVert.lerp(a, b, t));
      } else if (!inA && inB) {
        final denom = da - db;
        final t = denom.abs() > 1e-12 ? (da / denom).clamp(0.0, 1.0) : 0.0;
        result.add(_LocalClipVert.lerp(a, b, t));
        result.add(b);
      }
    }
    return result;
  }
}
