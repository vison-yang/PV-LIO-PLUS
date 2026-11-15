#!/usr/bin/env python3
"""Generate the i-Octree oracle without importing or executing native code.

All arithmetic expectations use Fraction and explicit integer RN/ties-to-even
rounding after EACH specified binary64 operation. Float inputs and expected
values are C++ integer bit patterns, never decimal floating-point literals.
The independent CPython Fraction.__float__ conversion cross-checks every RN64
result. --check is read-only and checks both generated files byte for byte.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import itertools
import json
from pathlib import Path
import random
import struct


SEED = 0x10C7EE
HOST_ROUND_CHECKS = 0
F32_MAX_BITS = 0x7F7FFFFF
F64_SIGN = 1 << 63
F32_SIGN = 1 << 31


def exact(bits: int, width: int) -> Fraction:
    """Decode a finite IEEE bit pattern, independently of host arithmetic."""
    fraction_bits, exponent_bits, bias = (23, 8, 127) if width == 32 else (52, 11, 1023)
    exponent = (bits >> fraction_bits) & ((1 << exponent_bits) - 1)
    significand = bits & ((1 << fraction_bits) - 1)
    if exponent == (1 << exponent_bits) - 1:
        raise ValueError("the exact oracle accepts only finite numbers")
    power = (1 - bias if exponent == 0 else exponent - bias) - fraction_bits
    if exponent:
        significand |= 1 << fraction_bits
    value = Fraction(significand << max(power, 0), 1 << max(-power, 0))
    return -value if bits >> (width - 1) else value


def rn_bits(value: Fraction, width: int) -> int:
    """Correctly round an arbitrary rational using integer quotient/remainder."""
    fraction_bits, exponent_bits, bias = (23, 8, 127) if width == 32 else (52, 11, 1023)
    sign = (1 << (width - 1)) if value < 0 else 0
    value = abs(value)
    if not value:
        return sign
    numerator, denominator = value.numerator, value.denominator
    exponent = numerator.bit_length() - denominator.bit_length()
    if ((numerator < (denominator << exponent)) if exponent >= 0
            else ((numerator << -exponent) < denominator)):
        exponent -= 1
    minimum_exponent = 1 - bias
    step_exponent = max(exponent, minimum_exponent) - fraction_bits
    if step_exponent >= 0:
        denominator <<= step_exponent
    else:
        numerator <<= -step_exponent
    significand, remainder = divmod(numerator, denominator)
    twice = remainder * 2
    if twice > denominator or (twice == denominator and significand & 1):
        significand += 1
    if significand < (1 << fraction_bits):
        return sign | significand  # Includes rounded underflow to signed zero.
    exponent = max(exponent, minimum_exponent)
    if significand == (1 << (fraction_bits + 1)):
        significand >>= 1
        exponent += 1
    if exponent > bias:
        return sign | (((1 << exponent_bits) - 1) << fraction_bits)
    return sign | ((exponent + bias) << fraction_bits) | (significand - (1 << fraction_bits))


def host64_bits(value: float) -> int:
    return struct.unpack(">Q", struct.pack(">d", value))[0]


def rn64(value: Fraction) -> int:
    global HOST_ROUND_CHECKS
    result = rn_bits(value, 64)
    try:
        host = host64_bits(float(value))
    except OverflowError:
        host = (F64_SIGN if value < 0 else 0) | 0x7FF0000000000000
    assert result == host, (value, hex(result), hex(host))
    HOST_ROUND_CHECKS += 1
    return result


def power2(exponent: int) -> Fraction:
    return Fraction(1 << max(exponent, 0), 1 << max(-exponent, 0))


def f32(value: Fraction | int) -> int:
    return rn_bits(Fraction(value), 32)


def f64(value: Fraction | int) -> int:
    return rn64(Fraction(value))


def promote(bits: int) -> int:
    return (F64_SIGN if bits == F32_SIGN else rn64(exact(bits, 32)))


def quantize(bits: int) -> int:
    return (F32_SIGN if bits == F64_SIGN else rn_bits(exact(bits, 64), 32))


def point32(x=0, y=0, z=0) -> tuple[int, int, int]:
    return tuple(f32(Fraction(value)) for value in (x, y, z))


def point64(x=0, y=0, z=0) -> tuple[int, int, int]:
    return tuple(f64(Fraction(value)) for value in (x, y, z))


def promoted(point: tuple[int, int, int]) -> tuple[int, int, int]:
    return tuple(promote(value) for value in point)


def difference(point: int, query: int) -> int:
    return rn64(exact(point, 32) - exact(query, 64)) & ~F64_SIGN


def squared_sum(deltas: list[int]) -> int:
    squares = [rn64(exact(delta, 64) ** 2) for delta in deltas]
    first = rn64(exact(squares[0], 64) + exact(squares[1], 64))
    return rn64(exact(first, 64) + exact(squares[2], 64))


def distance(point: tuple[int, int, int], query: tuple[int, int, int]) -> int:
    return squared_sum([difference(p, q) for p, q in zip(point, query)])


def radius_squared(radius: int) -> int:
    return rn64(exact(radius, 64) ** 2)


def bounds(lo, hi, query) -> tuple[int, int]:
    near, far = [], []
    for left, right, q in zip(lo, hi, query):
        assert exact(left, 32) <= exact(right, 32)
        dl, dr = difference(left, q), difference(right, q)
        near.append(dl if exact(q, 64) < exact(left, 32)
                    else dr if exact(q, 64) > exact(right, 32) else 0)
        far.append(max(dl, dr))  # Nonnegative binary64 bit order is numeric order.
    return squared_sum(near), squared_sum(far)


def adjacent64(bits: int, direction: int) -> int:
    """Next finite binary64 toward +/- infinity; inputs here are in float range."""
    if bits & ~F64_SIGN == 0:
        return 1 if direction > 0 else F64_SIGN | 1
    return bits + (1 if (direction > 0) == (bits >> 63 == 0) else -1)


def random32(rng: random.Random) -> int:
    while True:
        bits = rng.getrandbits(32)
        if (bits >> 23) & 0xFF != 0xFF:
            return bits


def random_query_axis(rng: random.Random) -> int:
    # Deliberately mixes double subnormals, non-float values and large values.
    exponent = rng.randint(-1074, 127)
    magnitude = Fraction((1 << 52) + rng.getrandbits(52), 1 << 52) * power2(exponent)
    maximum = exact(F32_MAX_BITS, 32)
    magnitude = min(magnitude, maximum)
    return rn64(-magnitude if rng.getrandbits(1) else magnitude)


def self_test_rounding() -> None:
    for width, fraction_bits, minimum in ((32, 23, -149), (64, 52, -1074)):
        one = rn_bits(Fraction(1), width)
        assert rn_bits(1 + power2(-fraction_bits - 1), width) == one
        assert rn_bits(1 + 3 * power2(-fraction_bits - 1), width) == one + 2
        assert rn_bits(power2(minimum - 1), width) == 0
        assert rn_bits(3 * power2(minimum - 1), width) == 2
        assert rn_bits(-power2(minimum - 1), width) == 1 << (width - 1)
    rng = random.Random(SEED ^ 0xC0DE)
    for _ in range(1000):
        exponent = rng.randint(-1100, 1000)
        value = Fraction(rng.getrandbits(80), rng.getrandbits(30) + 1) * power2(exponent)
        rn64(-value if rng.getrandbits(1) else value)
        bits = random32(rng)
        # Zero's sign is a representation issue, not a rational distinction.
        assert rn_bits(exact(bits, 32), 32) == (0 if bits == F32_SIGN else bits)
    assert promote(F32_SIGN) == F64_SIGN
    assert quantize(F64_SIGN) == F32_SIGN


def build_fixtures():
    rng = random.Random(SEED)
    scalar = []

    def add_scalar(name, point, query):
        scalar.append((name, point, query, distance(point, query)))

    add_scalar("leaking_root_2pow24", point32((1 << 24) + 2), point64(1 << 24))
    add_scalar("negative_translation_2pow24", point32(-(1 << 24) - 2), point64(-(1 << 24)))
    for sign in (1, -1):
        add_scalar("huge_translation_" + str(sign),
                   point32(sign * (power2(100) + power2(77)), power2(70), -power2(72)),
                   point64(sign * power2(100), power2(70) + power2(18), -power2(72)))
    maximum = exact(F32_MAX_BITS, 32)
    for point, query, name in (
        ((1, 0, 0), (0, 0, 0), "positive_float_subnormal"),
        ((F32_SIGN | 1, 2, F32_SIGN | 3), (0, 0, 0), "mixed_float_subnormals"),
        ((F32_SIGN, 0, F32_SIGN), (F64_SIGN, 0, F64_SIGN), "signed_zero"),
        ((F32_MAX_BITS, F32_MAX_BITS, F32_MAX_BITS), point64(-maximum, -maximum, -maximum), "opposite_fltmax"),
        ((F32_SIGN | F32_MAX_BITS, 0, F32_MAX_BITS), point64(maximum, maximum, -maximum), "mixed_fltmax"),
        (point32(1, 1, 1), (0x3FF0000000000001, 0x3FEFFFFFFFFFFFFF, 0x3FF0000000000002), "query_not_float"),
        (point32(1, 3 * power2(-28), 3 * power2(-28)), point64(), "left_associated_sum"),
        (point32(1, power2(-26), power2(-26)), point64(), "rounded_radius_equal"),
        (point32(), (1, F64_SIGN | 1, 2), "double_subnormal_query_squares_underflow"),
    ):
        add_scalar(name, point, query)
    for index in range(192):
        point = tuple(random32(rng) for _ in range(3))
        if index % 3 == 0:
            query = tuple(adjacent64(promote(p), 1 if rng.getrandbits(1) else -1) for p in point)
            # Do not step outside the documented query domain at +/-FLT_MAX.
            query = tuple(rn64(max(-maximum, min(maximum, exact(q, 64)))) for q in query)
        elif index % 3 == 1:
            query = tuple(random_query_axis(rng) for _ in range(3))
        else:
            query = tuple(rn64(exact(p, 32) + power2(rng.randint(-149, 50))) for p in point)
            query = tuple(rn64(max(-maximum, min(maximum, exact(q, 64)))) for q in query)
        add_scalar("random_scalar_%03d" % index, point, query)

    radius_values = [f64(1), f64(2), 0x3FFFFFFFFFFFFFFF, 0x4000000000000001,
                     0x3FF6A09E667F3BCC, 0x3FF6A09E667F3BCD, 0x3FF0000000000001,
                     f64(power2(-537)), promote(1), f64(maximum), f64(4 * maximum)]
    radius_values += [rn64(power2(rng.randint(-537, 128)) * Fraction(rng.randrange(16, 32), 16))
                      for _ in range(64)]
    radius_cases = [(value, radius_squared(value)) for value in radius_values]
    assert all(threshold > 0 and threshold < 0x7FF0000000000000 for _, threshold in radius_cases)
    invalid_radii = [0, F64_SIGN, f64(-1), 0x7FF0000000000000, 0xFFF0000000000000,
                     0x7FF8000000000001, 1, f64(power2(-538))]

    box_specs = [
        ("point_zero", point32(), point32(), point64()),
        ("signed_zero_box", (F32_SIGN, 0, F32_SIGN), (0, F32_SIGN, 0), (1, F64_SIGN | 1, 0)),
        ("leaking_root_box", point32(1 << 24), point32((1 << 24) + 2), point64(1 << 24)),
        ("negative_root_box", point32(-(1 << 24) - 2), point32(-(1 << 24)), point64(-(1 << 24))),
        ("full_float_domain", point32(-maximum, -maximum, -maximum), point32(maximum, maximum, maximum), point64()),
        ("full_float_domain_corner", point32(-maximum, -maximum, -maximum), point32(maximum, maximum, maximum), point64(maximum, -maximum, maximum)),
        ("subnormal_box", (F32_SIGN | 3, F32_SIGN | 2, F32_SIGN | 1), (3, 2, 1), point64()),
        ("inside_nonfloat", point32(1, -1, 0), point32(2, 1, 0), (0x3FF0000000000001, 1, 0)),
        ("just_below_endpoint", point32(1, 2, 3), point32(2, 3, 4), (0x3FEFFFFFFFFFFFFF, f64(2), f64(3))),
        ("just_above_endpoint", point32(1, 2, 3), point32(2, 3, 4), (0x4000000000000001, f64(3), f64(4))),
    ]
    for index in range(80):
        a, b = [tuple(random32(rng) for _ in range(3)) for _ in range(2)]
        lo = tuple(min(x, y, key=lambda bits: exact(bits, 32)) for x, y in zip(a, b))
        hi = tuple(max(x, y, key=lambda bits: exact(bits, 32)) for x, y in zip(a, b))
        if index % 4 == 0:
            query = tuple(rn64((exact(x, 32) + exact(y, 32)) / 2) for x, y in zip(lo, hi))
        elif index % 4 == 1:
            query = promoted(lo)
        elif index % 4 == 2:
            query = tuple(adjacent64(promote(x), 1) for x in hi)
        else:
            query = tuple(random_query_axis(rng) for _ in range(3))
        box_specs.append(("random_bounds_%03d" % index, lo, hi, query))
    box_points, box_cases = [], []
    for name, lo, hi, query in box_specs:
        points = list(itertools.product(*zip(lo, hi)))
        for numerator in (1, 2, 3):
            points.append(tuple(rn_bits((exact(left, 32) * (4 - numerator) + exact(right, 32) * numerator) / 4, 32)
                                for left, right in zip(lo, hi)))
        lower, upper = bounds(lo, hi, query)
        first = len(box_points)
        for point in points:
            value = distance(point, query)
            assert lower <= value <= upper, name
            box_points.append((point, value))
        box_cases.append((name, lo, hi, query, lower, upper, first, len(points)))

    scenes, scene_points, queries, query_distances = [], [], [], []

    def add_scene(name, points, requests, double_inputs=False):
        scene_index = len(scenes)
        inputs = points if double_inputs else [promoted(point) for point in points]
        stored = [tuple(quantize(axis) for axis in point) for point in inputs]
        point_first = len(scene_points)
        scene_points.extend(zip(inputs, stored))
        scenes.append((name, point_first, len(points)))
        for label, query, k, radius in requests:
            threshold = radius_squared(radius)
            assert threshold > 0 and threshold < 0x7FF0000000000000
            distance_first = len(query_distances)
            query_distances.extend(distance(point, query) for point in stored)
            queries.append((name + "_" + label, scene_index, query, k, radius, threshold, distance_first, len(points)))

    add_scene("empty", [], [("zero_k", point64(), 0, f64(1)), ("positive_k", point64(), 7, f64(1))])
    for sign, name in ((1, "positive_2pow24"), (-1, "negative_2pow24")):
        base = sign * (1 << 24)
        points = [point32(base), point32(base + sign * 2)]
        requests = [("leak_radius", point64(base), 5, f64(Fraction(19, 10))),
                    ("strict_equal", point64(base), 1, f64(2)),
                    ("just_larger", point64(base), 2, 0x4000000000000001),
                    ("midpoint_query", point64(base + sign), 1, f64(1))]
        add_scene(name, points, requests)
    for sign in (1, -1):
        base, step = sign * power2(100), power2(77)
        points = [point32(base + sign * i * step, (i % 3) * step, -(i % 2) * step) for i in range(9)]
        add_scene("huge_translation_%s" % sign, points,
                  [("origin", point64(base), 20, f64(Fraction(19, 10) * step)),
                   ("nonfloat_query", point64(base + sign * step / 4, step / 8), 3, f64(3 * step)),
                   ("all", point64(base), 4, f64(20 * step))])
    extreme_points = [point32(maximum, maximum, maximum), point32(-maximum, -maximum, -maximum),
                      point32(maximum, -maximum, 0), point32(-maximum, maximum, 0), point32(), (F32_SIGN, 0, F32_SIGN)]
    add_scene("extreme_points", extreme_points,
              [("all_large_distances", point64(maximum, maximum, maximum), 100, f64(4 * maximum)),
               ("origin", point64(), 2, f64(maximum)),
               ("negative_corner", point64(-maximum, -maximum, -maximum), 4, f64(maximum))])
    tiny_points = [(1, 0, 0), (2, 0, 0), (F32_SIGN | 1, 0, 0), (0, 0, 0), (F32_SIGN, 0, F32_SIGN), (0, 1, 1)]
    add_scene("subnormal_points", tiny_points,
              [("strict_float_subnormal", point64(), 10, promote(1)),
               ("double_subnormal_threshold", point64(), 2, f64(power2(-537))),
               ("signed_zero_query", (F64_SIGN, F64_SIGN, 0), 0, promote(2))])
    add_scene("nonfloat_query", [point32(1, 1, 1), point32(1 + power2(-23), 1, 1), point32(1, 1 + power2(-23), 1)],
              [("one_double_ulp", (0x3FF0000000000001, f64(1), f64(1)), 1, f64(power2(-52))),
               ("within_half_float_ulp", point64(1 + power2(-25), 1 + power2(-26), 1), 8, f64(power2(-22)))])
    add_scene("float_distance_ties_but_double_orders", [point32(1, power2(-13), 0), point32(1), point32(-1), point32(1)],
              [("nearest", point64(), 1, f64(1)), ("ties", point64(), 3, 0x3FF0000000000001),
               ("fewer_than_k", point64(), 10, f64(2)), ("zero_k", point64(), 0, f64(2))])
    add_scene("left_associated_metric", [point32(1, 3 * power2(-28), 3 * power2(-28)), point32(1), point32(0, 1, 1)],
              [("nearest", point64(), 3, f64(1)), ("rounded_sum", point64(), 2, 0x3FF0000000000001)])
    add_scene("rounded_radius_threshold", [point32(1, power2(-26), power2(-26)), point32(-1, power2(-26), power2(-26)), point32()],
              [("real_inside_but_computed_equal", point64(), 3, 0x3FF0000000000001),
               ("sqrt2_below", point64(), 3, 0x3FF6A09E667F3BCC),
               ("sqrt2_above", point64(), 3, 0x3FF6A09E667F3BCD)])
    quantization_inputs = [point64(1 + power2(-24)), point64(1 + 3 * power2(-24)), point64(-(1 + power2(-24))),
                           point64((1 << 24) + 1), point64(-(1 << 24) - 1),
                           point64(power2(-150)), point64(-power2(-150)), point64(3 * power2(-150)),
                           (F64_SIGN, 0, F64_SIGN)]
    add_scene("map_quantizes_once", quantization_inputs,
              [("origin", point64(), 20, f64(2)), ("large", point64(1 << 24), 5, f64(2))], double_inputs=True)
    for label, wide in (("random_compact", False), ("random_wide", True)):
        points = ([tuple(random32(rng) for _ in range(3)) for _ in range(36)] if wide else
                  [point32(*(Fraction(rng.randint(-2048, 2048), 128) for _ in range(3))) for _ in range(36)])
        requests = []
        for index in range(24):
            if index % 3 == 0:
                query = promoted(points[index % len(points)])
            elif index % 3 == 1:
                query = tuple(random_query_axis(rng) for _ in range(3)) if wide else point64(
                    *(Fraction(rng.randint(-2048, 2048), 128) + power2(-40) for _ in range(3)))
            else:
                query = point64()
            k = (0, 1, 3, 17, 36, 50)[index % 6]
            radius = f64(4 * maximum) if wide and index % 2 else f64(4 + index)
            requests.append(("query_%02d" % index, query, k, radius))
        add_scene(label, points, requests)

    # Explicit witnesses that the fixture is testing the chosen computational
    # metric, rather than silently claiming exact-real or binary32 distances.
    witness = distance(point32(1, power2(-26), power2(-26)), point64())
    radius = 0x3FF0000000000001
    assert radius_squared(radius) == witness
    assert exact(radius, 64) ** 2 > exact(witness, 64)
    farther = distance(point32(1, power2(-13)), point64())
    nearer = distance(point32(1), point64())
    assert farther > nearer and rn_bits(exact(farther, 64), 32) == rn_bits(exact(nearer, 64), 32)
    left = distance(point32(1, 3 * power2(-28), 3 * power2(-28)), point64())
    other = rn64(Fraction(1) + exact(rn64(18 * power2(-56)), 64))
    assert left != other
    return scalar, radius_cases, invalid_radii, box_points, box_cases, scene_points, scenes, queries, query_distances


def cpp_bits(bits: int, width: int) -> str:
    return "0x%0*x%s" % (width // 4, bits, "u" if width == 32 else "ULL")


def cpp_point(point, width: int) -> str:
    return "{{" + ", ".join(cpp_bits(axis, width) for axis in point) + "}}"


def render(data):
    scalar, radii, invalid_radii, box_points, boxes, points, scenes, queries, query_distances = data
    lines = ["// Generated by tests/generate_i_octree_numeric_oracle.py; do not edit.",
             "// Independent Fraction + integer RN/ties-to-even; seed 0x%06x." % SEED,
             "#pragma once", "#include <cstddef>", "#include <cstdint>",
             "namespace i_octree_numeric_fixture {",
             "struct FloatPoint { std::uint32_t xyz[3]; };",
             "struct DoublePoint { std::uint64_t xyz[3]; };",
             "struct ScalarCase { const char *name; FloatPoint point; DoublePoint query; std::uint64_t distance; };",
             "struct RadiusCase { std::uint64_t radius, threshold; };",
             "struct BoxPoint { FloatPoint point; std::uint64_t distance; };",
             "struct BoxCase { const char *name; FloatPoint lo, hi; DoublePoint query; std::uint64_t lower, upper; std::size_t first, count; };",
             "struct TreePoint { DoublePoint input; FloatPoint stored; };",
             "struct Scene { const char *name; std::size_t first, count; };",
             "struct Query { const char *name; std::size_t scene; DoublePoint point; int k; std::uint64_t radius, threshold; std::size_t first, count; };",
             "inline constexpr ScalarCase scalar_cases[] = {"]
    for name, point, query, value in scalar:
        lines.append('    {"%s", %s, %s, %s},' % (name, cpp_point(point, 32), cpp_point(query, 64), cpp_bits(value, 64)))
    lines += ["};", "inline constexpr RadiusCase radius_cases[] = {"]
    for radius, threshold in radii:
        lines.append("    {%s, %s}," % (cpp_bits(radius, 64), cpp_bits(threshold, 64)))
    lines += ["};", "inline constexpr std::uint64_t invalid_radii[] = {"]
    lines += ["    " + cpp_bits(value, 64) + "," for value in invalid_radii]
    lines += ["};", "inline constexpr BoxPoint box_points[] = {"]
    lines += ["    {%s, %s}," % (cpp_point(point, 32), cpp_bits(value, 64)) for point, value in box_points]
    lines += ["};", "inline constexpr BoxCase box_cases[] = {"]
    for name, lo, hi, query, lower, upper, first, count in boxes:
        lines.append('    {"%s", %s, %s, %s, %s, %s, %d, %d},' %
                     (name, cpp_point(lo, 32), cpp_point(hi, 32), cpp_point(query, 64), cpp_bits(lower, 64), cpp_bits(upper, 64), first, count))
    lines += ["};", "inline constexpr TreePoint tree_points[] = {"]
    lines += ["    {%s, %s}," % (cpp_point(input_point, 64), cpp_point(stored, 32)) for input_point, stored in points]
    lines += ["};", "inline constexpr Scene scenes[] = {"]
    lines += ['    {"%s", %d, %d},' % scene for scene in scenes]
    lines += ["};", "inline constexpr Query queries[] = {"]
    for name, scene, point, k, radius, threshold, first, count in queries:
        lines.append('    {"%s", %d, %s, %d, %s, %s, %d, %d},' %
                     (name, scene, cpp_point(point, 64), k, cpp_bits(radius, 64), cpp_bits(threshold, 64), first, count))
    lines += ["};", "inline constexpr std::uint64_t query_distances[] = {"]
    for start in range(0, len(query_distances), 4):
        lines.append("    " + ", ".join(cpp_bits(value, 64) for value in query_distances[start:start + 4]) + ",")
    lines += ["};", "}  // namespace i_octree_numeric_fixture", ""]
    header = "\n".join(lines)
    manifest = {
        "format": 1,
        "seed": "0x%06x" % SEED,
        "oracle": "Fraction exact rational arithmetic; integer IEEE RN/ties-to-even after each operation",
        "metric": "abs(RN(float_point-double_query)); RN(square) per axis; RN(RN(s0+s1)+s2)",
        "radius": "T=RN(r*r); membership D<T; computed metric, not exact-real distance",
        "scalar_cases": len(scalar), "radius_cases": len(radii), "invalid_radii": len(invalid_radii),
        "box_cases": len(boxes), "box_point_checks": len(box_points), "scenes": len(scenes),
        "tree_points": len(points), "tree_queries": len(queries), "tree_point_distance_checks": len(query_distances),
        "fraction_float_crosschecks": HOST_ROUND_CHECKS,
        "header_sha256": hashlib.sha256(header.encode()).hexdigest(),
        "witnesses": ["2^24 missing-root-coverage counterexample", "binary32 distance tie with distinct binary64 ranks",
                      "RN radius threshold equals D although exact r*r>D", "left-associated sum differs from regrouping",
                      "double query not quantized to float", "signed-zero and ties-to-even map quantization"],
    }
    return header, json.dumps(manifest, indent=2, sort_keys=True) + "\n", manifest


def main() -> None:
    if not __debug__:
        raise SystemExit("run without Python -O: oracle self-checks must remain enabled")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="self-test and verify existing fixtures without writing")
    arguments = parser.parse_args()
    self_test_rounding()
    header, metadata, manifest = render(build_fixtures())
    directory = Path(__file__).resolve().parent / "data"
    outputs = {directory / "i_octree_numeric_oracle.h": header,
               directory / "i_octree_numeric_oracle.json": metadata}
    if arguments.check:
        for path, content in outputs.items():
            if not path.exists() or path.read_text() != content:
                raise SystemExit("fixture missing or stale: %s; regenerate with this script" % path)
    else:
        directory.mkdir(exist_ok=True)
        for path, content in outputs.items():
            path.write_text(content)
    print(json.dumps({"status": "checked" if arguments.check else "generated", **manifest}, sort_keys=True))


if __name__ == "__main__":
    main()
