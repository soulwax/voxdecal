import 'dart:math' as math;

/// Immutable 2D vector for UV mapping in [voxdecal].
final class DecalVec2 {
  final double x;
  final double y;

  const DecalVec2(this.x, this.y);

  static const DecalVec2 zero = DecalVec2(0.0, 0.0);
  static const DecalVec2 one = DecalVec2(1.0, 1.0);

  DecalVec2 operator +(DecalVec2 o) => DecalVec2(x + o.x, y + o.y);
  DecalVec2 operator -(DecalVec2 o) => DecalVec2(x - o.x, y - o.y);
  DecalVec2 operator *(double s) => DecalVec2(x * s, y * s);

  static DecalVec2 lerp(DecalVec2 a, DecalVec2 b, double t) =>
      DecalVec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);

  @override
  String toString() => 'DecalVec2($x, $y)';
}

/// Immutable 3D vector for [voxdecal].
final class DecalVec3 {
  final double x;
  final double y;
  final double z;

  const DecalVec3(this.x, this.y, this.z);

  static const DecalVec3 zero = DecalVec3(0.0, 0.0, 0.0);
  static const DecalVec3 unitX = DecalVec3(1.0, 0.0, 0.0);
  static const DecalVec3 unitY = DecalVec3(0.0, 1.0, 0.0);
  static const DecalVec3 unitZ = DecalVec3(0.0, 0.0, 1.0);

  DecalVec3 operator +(DecalVec3 o) => DecalVec3(x + o.x, y + o.y, z + o.z);
  DecalVec3 operator -(DecalVec3 o) => DecalVec3(x - o.x, y - o.y, z - o.z);
  DecalVec3 operator -() => DecalVec3(-x, -y, -z);
  DecalVec3 operator *(double s) => DecalVec3(x * s, y * s, z * s);
  DecalVec3 operator /(double s) => DecalVec3(x / s, y / s, z / s);

  double dot(DecalVec3 o) => x * o.x + y * o.y + z * o.z;

  DecalVec3 cross(DecalVec3 o) => DecalVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  DecalVec3 get normalized {
    final lenSq = lengthSquared;
    if (lenSq <= 1e-16) return unitY;
    final inv = 1.0 / math.sqrt(lenSq);
    return DecalVec3(x * inv, y * inv, z * inv);
  }

  double distanceTo(DecalVec3 o) => (this - o).length;

  static DecalVec3 lerp(DecalVec3 a, DecalVec3 b, double t) => DecalVec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      );

  @override
  String toString() => 'DecalVec3($x, $y, $z)';
}

/// Unit quaternion used to orient decal projectors and rings.
final class DecalQuat {
  final double x;
  final double y;
  final double z;
  final double w;

  const DecalQuat(this.x, this.y, this.z, this.w);

  static const DecalQuat identity = DecalQuat(0.0, 0.0, 0.0, 1.0);

  factory DecalQuat.fromAxisAngle(DecalVec3 axis, double angleRad) {
    final norm = axis.normalized;
    if (norm.lengthSquared <= 1e-12) return identity;
    final half = angleRad * 0.5;
    final s = math.sin(half);
    return DecalQuat(norm.x * s, norm.y * s, norm.z * s, math.cos(half));
  }

  /// Builds an orientation whose local `-Z` points along [forward].
  factory DecalQuat.lookDirection(DecalVec3 forward, [DecalVec3 up = DecalVec3.unitY]) {
    if (forward.lengthSquared <= 1e-12) return identity;
    final z = -forward.normalized;
    var x = up.cross(z);
    if (x.lengthSquared <= 1e-10) {
      x = (z.y.abs() < 0.99 ? DecalVec3.unitY : DecalVec3.unitX).cross(z);
    }
    x = x.normalized;
    final y = z.cross(x);
    final m00 = x.x, m01 = y.x, m02 = z.x;
    final m10 = x.y, m11 = y.y, m12 = z.y;
    final m20 = x.z, m21 = y.z, m22 = z.z;
    final trace = m00 + m11 + m22;
    if (trace > 0.0) {
      final s = math.sqrt(trace + 1.0) * 2.0;
      return DecalQuat((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s);
    }
    if (m00 > m11 && m00 > m22) {
      final s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0;
      return DecalQuat(0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
    }
    if (m11 > m22) {
      final s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0;
      return DecalQuat((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s);
    }
    final s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0;
    return DecalQuat((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s);
  }

  DecalQuat get normalized {
    final lenSq = x * x + y * y + z * z + w * w;
    if (lenSq <= 1e-16) return identity;
    final inv = 1.0 / math.sqrt(lenSq);
    return DecalQuat(x * inv, y * inv, z * inv, w * inv);
  }

  DecalQuat get conjugate => DecalQuat(-x, -y, -z, w);

  DecalVec3 rotate(DecalVec3 v) {
    final qv = DecalVec3(x, y, z);
    final t = qv.cross(v) * 2.0;
    return v + t * w + qv.cross(t);
  }

  @override
  String toString() => 'DecalQuat($x, $y, $z, $w)';
}
