import 'package:test/test.dart';
import 'package:voxdecal/voxdecal.dart';

void main() {
  group('DecalBoxProjector', () {
    test('clips projected quad cleanly across box faces, offsets normals, and culls steep walls', () {
      final target = DecalSourceMesh.box(size: const DecalVec3(2.0, 2.0, 2.0));
      // Project downward (-Y) across the top edge of the box (y = +1.0, spanning x=[0.4..1.4] overhanging side wall).
      final rot = DecalQuat.lookDirection(const DecalVec3(0.0, -1.0, 0.0), DecalVec3.unitZ);
      final res = DecalBoxProjector.project(
        target: target,
        box: DecalBoxSpec(
          center: const DecalVec3(0.7, 1.0, 0.0),
          rotation: rot,
          size: const DecalVec3(1.0, 1.0, 0.6),
          maxNormalAngleDegrees: 50.0,
          surfaceOffset: 0.01,
        ),
      );

      expect(res.isEmpty, isFalse);
      expect(res.triangleCount, greaterThanOrEqualTo(2));
      // All kept vertices must lie on top face (y == 1.01) clipped inside x <= 1.0, steep x=1.0 wall culled.
      for (final v in res.vertices) {
        expect(v.position.y, closeTo(1.01, 1e-4));
        expect(v.position.x, inInclusiveRange(0.2 - 1e-4, 1.0 + 1e-4));
        expect(v.uv.x, inInclusiveRange(-1e-4, 1.0 + 1e-4));
        expect(v.uv.y, inInclusiveRange(-1e-4, 1.0 + 1e-4));
      }
    });
  });

  group('SkidRibbonBuilder', () {
    test('builds segmented strips with break gaps and fades out old tracks over time', () {
      final ribbon = SkidRibbonBuilder(
        defaultWidth: 0.3,
        minSegmentDistance: 0.2,
        lifetimeSeconds: 5.0,
        fadeDurationSeconds: 1.0,
      );

      for (var i = 0; i < 6; i++) {
        ribbon.addSample(position: DecalVec3(i * 0.5, 0.0, 0.0));
        ribbon.advance(0.1);
      }
      ribbon.breakStrip();
      for (var i = 0; i < 4; i++) {
        ribbon.addSample(position: DecalVec3(4.0 + i * 0.5, 0.0, 0.0));
        ribbon.advance(0.1);
      }

      final mesh = ribbon.buildMesh();
      expect(mesh.isEmpty, isFalse);
      expect(mesh.triangleCount, (5 + 3) * 2); // 5 quads + 3 quads across gap

      ribbon.advance(6.0);
      expect(ribbon.sampleCount, 0);
      expect(ribbon.buildMesh().isEmpty, isTrue);
    });
  });

  group('DecalPoolManager', () {
    test('enforces ring-buffer capacity FIFO eviction, proximity cull, and lifetime expiry', () {
      final pool = DecalPoolManager(
        maxCapacity: 3,
        mergeCullRadius: 0.2,
        defaultLifetimeSeconds: 10.0,
        defaultFadeDurationSeconds: 2.0,
      );

      final first = pool.spawn(
        position: DecalVec3.zero,
        normal: DecalVec3.unitY,
      );
      expect(first, isNotNull);

      // Duplicate stamp inside mergeCullRadius is culled.
      final dup = pool.spawn(
        position: const DecalVec3(0.05, 0.0, 0.05),
        normal: DecalVec3.unitY,
      );
      expect(dup, isNull);
      expect(pool.mergeCulledCount, 1);

      pool.spawn(position: const DecalVec3(2.0, 0.0, 0.0), normal: DecalVec3.unitY);
      pool.spawn(position: const DecalVec3(4.0, 0.0, 0.0), normal: DecalVec3.unitY);
      pool.spawn(position: const DecalVec3(6.0, 0.0, 0.0), normal: DecalVec3.unitY);

      expect(pool.activeCount, 3);
      expect(pool.evictedCount, 1);

      pool.tick(9.0);
      expect(pool.entries.first.opacity, closeTo(0.5, 1e-4));

      pool.tick(1.5);
      expect(pool.activeCount, 0);
    });
  });
}
